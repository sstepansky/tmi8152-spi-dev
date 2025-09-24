#ifndef __TMI8152_SPI_DEV_H__
#define __TMI8152_SPI_DEV_H__

static int tmi8152_cdev_open(struct inode *inode, struct file *fd);
static ssize_t tmi8152_cdev_read(struct file *filp, char __user * user_buf,
                                 size_t len, loff_t * off);
static ssize_t tmi8152_cdev_write(struct file *filp,
                                  const char __user * buff, size_t count,
                                  loff_t * offp);
static int tmi8152_cdev_release(struct inode *inode, struct file *fd);

static int tmi8152_spi_read(int addr, int *value);
static int tmi8152_spi_write(int addr, int value);

static int set_ir_cut(int val);
static int tmi8152_spi_status(int status);
static int tmi8152_spi_probe(struct spi_device *spi);
static int tmi8152_spi_remove(struct spi_device *spi);

int __init tmi8152_mod_init(void);
void __exit tmi8152_mod_exit(void);

#endif
