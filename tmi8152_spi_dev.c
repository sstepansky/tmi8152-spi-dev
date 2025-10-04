#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include "tmi8152_spi_dev.h"


dev_t device_number;
static struct spi_device *sdev = NULL;

struct cdev tmi8152_cdev;
struct class *class_tmi8152;

static struct mutex lock;
static char dev_buff[1];


static int tmi8152_cdev_open(struct inode *inode, struct file *fd)
{
    pr_debug("[%s] Opened TMI8152 character device\n", __func__);
    return 0;
}

static int tmi8152_cdev_release(struct inode *inode, struct file *fd)
{
    pr_debug("[%s] Closed TMI8152 character device\n", __func__);
    return 0;
}

static ssize_t tmi8152_cdev_read(struct file *filp, char __user *user_buf,
                                 size_t len, loff_t *off)
{
    int ret, delta, max;
    char temp_buff[1];

    pr_debug("[%s] Reading data from device\n", __func__);

    max =
        (len + *off) < sizeof(dev_buff) ? len : (sizeof(dev_buff) - *off);

    if (*off >= sizeof(dev_buff))
        return 0;

    mutex_lock(&lock);
    memcpy(temp_buff, &dev_buff[*off], max);
    mutex_unlock(&lock);

    ret = copy_to_user(user_buf, temp_buff, max);
    delta = max - ret;
    if (ret)
        pr_warn("[%s] Copied only %d bytes!\n", __func__, delta);

    *off += delta;

    return delta;
}

static ssize_t tmi8152_cdev_write(struct file *filp,
                                  const char __user *buff, size_t count,
                                  loff_t *offp)
{
    int ret;
    char temp_buff[1];

    pr_debug("[%s] Writing data to device\n", __func__);

    if (count > 1) {
        pr_err("[%s] Got %d bytes. Only one byte will be read\n",
               __func__, count);
        return -EINVAL;
    }

    ret = copy_from_user(&temp_buff, buff, 1);
    if (ret != 0) {
        pr_err("[%s] Error copying data from user\n", __func__);
        return -EFAULT;
    }

    /* Validate input */
    if (temp_buff[0] != '0' && temp_buff[0] != '1') {
        pr_err("[%s] Unrecognized value!\n", __func__);
        return -EINVAL;
    }

    /* Check if already in requested mode */
    mutex_lock(&lock);
    if (temp_buff[0] == dev_buff[0]) {
        mutex_unlock(&lock);
        pr_debug("[%s] Already in mode %c, skipping\n", __func__,
                 temp_buff[0]);
        return count;
    }
    mutex_unlock(&lock);

    /* Execute IR filter change */
    if (temp_buff[0] == '1') {
        ret = set_ir_cut(1);
    } else {
        ret = set_ir_cut(0);
    }

    if (ret < 0) {
        pr_err("[%s] Failed to set IR cut mode\n", __func__);
        return ret;
    }

    /* Update state after successful change */
    mutex_lock(&lock);
    dev_buff[0] = temp_buff[0];
    mutex_unlock(&lock);

    return count;
}

static struct file_operations tmi8152_cdev_fops = {
    .owner = THIS_MODULE,
    .open = tmi8152_cdev_open,
    .read = tmi8152_cdev_read,
    .write = tmi8152_cdev_write,
    .release = tmi8152_cdev_release,
};

static int set_ir_cut(int val)
{
    int ret_1;
    int ret_2;

    int cmd;

    if (val == 0) {
        cmd = 0x86;
    } else if (val == 1) {
        cmd = 0x8a;
    } else {
        pr_err("[%s] Unrecognized value\n", __func__);
        return -EINVAL;
    }

    ret_1 = tmi8152_spi_status(1);
    if (ret_1 < 0) {
        pr_err("[%s] Error setting SPI status!\n", __func__);
        return ret_1;
    }

    ret_1 = tmi8152_spi_write(0x91, cmd);
    msleep(180);
    ret_2 = tmi8152_spi_write(0x91, 0x0);

    if (ret_1 < 0 || ret_2 < 0) {
        pr_err("[%s] Error in IR cut!\n", __func__);
        return -EIO;
    }

    ret_1 = tmi8152_spi_status(0);
    if (ret_1 < 0) {
        pr_err("[%s] Error clearing SPI status!\n", __func__);
        return ret_1;
    }

    return 0;
}

static int tmi8152_spi_status(int status)
{
    int ret;

    ret = tmi8152_spi_write(0x82, 0x03);
    if (ret < 0)
        return ret;

    if (status == 0) {
        ret = tmi8152_spi_write(0x82, 0xc1);
    } else if (status == 1) {
        ret = tmi8152_spi_write(0x82, 0x8b);
    }

    return ret;
}

static int tmi8152_spi_read(int addr, int *value)
{
    struct spi_message message;
    struct spi_transfer transfer[2];
    unsigned char rbuf[4] = { 0, 0, 0, 0 };
    unsigned char wbuf[4] = { 0, 0, 0, 0 };
    int ret = 0;

    spi_message_init(&message);
    memset(&transfer, 0, sizeof(transfer));

    wbuf[0] = (addr | 0x40) & 0xff;

    transfer[0].tx_buf = wbuf;
    transfer[0].len = 1;
    spi_message_add_tail(&transfer[0], &message);

    transfer[1].rx_buf = rbuf;
    transfer[1].len = 1;
    transfer[1].cs_change = 1;
    spi_message_add_tail(&transfer[1], &message);

    ret = spi_sync(sdev, &message);

    if (ret) {
        pr_err("[%s] Error reading message from SPI device!\n", __func__);
        return ret;
    }
    *value = rbuf[0];

    return ret;
}

int tmi8152_spi_write(int addr, int value)
{
    int ret = 0;
    struct spi_message message;
    struct spi_transfer transfer;
    char wbuf[2];

    spi_message_init(&message);
    memset(&transfer, 0, sizeof(transfer));

    wbuf[0] = addr & 0xff;
    wbuf[1] = value & 0xff;

    transfer.tx_buf = wbuf;
    transfer.len = 2;
    transfer.cs_change = 1;
    spi_message_add_tail(&transfer, &message);

    ret = spi_sync(sdev, &message);
    if (ret) {
        pr_err("[%s] Error writing message to SPI device!\n", __func__);
        ret = -EIO;
    }

    return ret;
}

static int tmi8152_spi_probe(struct spi_device *spi)
{
    int ret = 0;
    int ret_1 = 0;
    int ret_2 = 0;
    int value = 0xff;
    unsigned int id_hi, id_lo;

    spi->mode = 0;
    sdev = spi;

    pr_info("Probing TMI8152 driver\n");
    tmi8152_spi_read(0x00, &id_hi);
    tmi8152_spi_read(0x01, &id_lo);
    if (id_hi == 0x81 && id_lo == 0x50) {
        pr_info("Chip ID is 0x%02x%02x\n", id_hi, id_lo);
    } else {
        pr_err("Chip ID 0x%02x%02x is not TMI8152!\n", id_hi, id_lo);
        return -ENODEV;
    }

    ret_1 = tmi8152_spi_write(0x82, 0x03);
    ret_2 = tmi8152_spi_write(0x82, 0x8b);
    if (ret_1 < 0 || ret_2 < 0) {
        pr_err("[%s] Error writing to TMI8152: %d, %d\n", __func__,
               ret_1, ret_2);
        return -EIO;
    }

    set_ir_cut(1);
    msleep(120);
    set_ir_cut(0);

    dev_buff[0] = '0';

    return 0;
}

static int tmi8152_spi_remove(struct spi_device *spi)
{
    return 0;
}

static struct spi_device_id sdev_id_table[] = {
    {
     .name = "tmi8152",
      },
    { }
};

MODULE_DEVICE_TABLE(spi, sdev_id_table);

static struct spi_driver tmi8152_driver = {
    .driver = {
               .name = "tmi8152",
               .owner = THIS_MODULE,
                },
    .id_table = sdev_id_table,
    .probe = tmi8152_spi_probe,
    .remove = tmi8152_spi_remove,
    .shutdown = NULL,
};

struct spi_board_info board_info_tmi8152[] = {
    [0] = {
           .modalias = "tmi8152",
           .bus_num = 0,
           .chip_select = 0,
           .max_speed_hz = 4000000,
            },
};

int __init tmi8152_mod_init(void)
{
    struct spi_master *spi_master_tmi8152;
    struct device *dev;
    int ret;

    pr_info("Registering TMI8152 driver\n");

    mutex_init(&lock);

    ret = alloc_chrdev_region(&device_number, 0, 1, "tmi8152_ir_cut");
    if (ret < 0) {
        pr_err("Failed to allocate chrdev region: %d\n", ret);
        return ret;
    }

    cdev_init(&tmi8152_cdev, &tmi8152_cdev_fops);
    tmi8152_cdev.owner = THIS_MODULE;
    ret = cdev_add(&tmi8152_cdev, device_number, 1);
    if (ret < 0) {
        pr_err("Failed to add cdev: %d\n", ret);
        goto err_cdev_add;
    }

    spi_master_tmi8152 = spi_busnum_to_master(board_info_tmi8152->bus_num);
    if (!spi_master_tmi8152) {
        pr_err("Failed to get SPI master\n");
        ret = -ENODEV;
        goto err_spi_master;
    }

    sdev = spi_new_device(spi_master_tmi8152, board_info_tmi8152);
    if (!sdev) {
        pr_err("Failed to create SPI device\n");
        ret = -ENOMEM;
        goto err_spi_device;
    }

    class_tmi8152 = class_create(THIS_MODULE, "tmi8152_ir_cut");
    if (IS_ERR(class_tmi8152)) {
        pr_err("Failed to create class\n");
        ret = PTR_ERR(class_tmi8152);
        goto err_class;
    }

    dev = device_create(class_tmi8152, NULL, device_number, NULL,
                        "tmi8152_ir_cut");
    if (IS_ERR(dev)) {
        pr_err("Failed to create device\n");
        ret = PTR_ERR(dev);
        goto err_device;
    }

    ret = spi_register_driver(&tmi8152_driver);
    if (ret < 0) {
        pr_err("Failed to register SPI driver: %d\n", ret);
        goto err_register;
    }

    pr_info("TMI8152 driver registered successfully\n");
    return 0;

  err_register:
    device_destroy(class_tmi8152, device_number);
  err_device:
    class_destroy(class_tmi8152);
  err_class:
    spi_unregister_device(sdev);
  err_spi_device:
  err_spi_master:
    cdev_del(&tmi8152_cdev);
  err_cdev_add:
    unregister_chrdev_region(device_number, 1);
    return ret;
}

void __exit tmi8152_mod_exit(void)
{
    pr_info("Unregistering TMI8152 driver\n");

    device_destroy(class_tmi8152, device_number);
    class_destroy(class_tmi8152);
    cdev_del(&tmi8152_cdev);
    unregister_chrdev_region(device_number, 1);

    spi_unregister_driver(&tmi8152_driver);
    if (sdev)
        spi_unregister_device(sdev);
}


module_init(tmi8152_mod_init);
module_exit(tmi8152_mod_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("sstepansky");
MODULE_DESCRIPTION("A simple driver for the TMI8152 chip");
