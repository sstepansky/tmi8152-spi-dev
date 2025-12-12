#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include "tmi8152_spi_dev.h"
#include "tmi8152_regs.h"
#include "motor.h"

/* Channel register mapping arrays (indexed by channel number 0 or 1) */
static const u8 ch_ctrl[] = {TMI8152_CH1_CTRL, TMI8152_CH2_CTRL};
static const u8 ch_dir[] = {TMI8152_CH1_DIR, TMI8152_CH2_DIR};
static const u8 ch_speed[] = {TMI8152_CH1_SPEED, TMI8152_CH2_SPEED};
static const u8 ch_pos_l[] = {TMI8152_CH1_POS_L, TMI8152_CH2_POS_L};
static const u8 ch_pos_h[] = {TMI8152_CH1_POS_H, TMI8152_CH2_POS_H};
static const u8 ch_tgt_l[] = {TMI8152_CH1_TGT_L, TMI8152_CH2_TGT_L};
static const u8 ch_tgt_h[] = {TMI8152_CH1_TGT_H, TMI8152_CH2_TGT_H};
static const u8 ch_phase_l[] = {TMI8152_CH1_PHASE_L, TMI8152_CH2_PHASE_L};
static const u8 ch_phase_h[] = {TMI8152_CH1_PHASE_H, TMI8152_CH2_PHASE_H};

/* Speed Profile Table */
typedef struct {
	u8 multiplier;
	u8 base;
	u16 threshold;
} speed_entry_t;

/* Complete 142-entry speed profile table */
static const speed_entry_t speed_profile_table[142] = {
	{7, 25, 32}, {7, 24, 34}, {7, 23, 35}, {7, 22, 37},
	{7, 21, 38}, {7, 20, 40}, {7, 19, 42}, {7, 18, 44},
	{7, 17, 47}, {7, 16, 50}, {6, 31, 53}, {6, 30, 55},
	{7, 14, 56}, {6, 28, 58}, {7, 13, 61}, {6, 26, 63},
	{7, 12, 65}, {6, 24, 68}, {7, 11, 71}, {6, 22, 74},
	{7, 10, 77}, {6, 20, 81}, {7, 9, 85}, {6, 18, 89},
	{7, 8, 94}, {6, 16, 100}, {7, 7, 106}, {5, 30, 110},
	{6, 14, 113}, {5, 28, 117}, {7, 6, 122}, {5, 26, 126},
	{6, 12, 131}, {5, 24, 136}, {7, 5, 142}, {5, 22, 148},
	{6, 10, 155}, {5, 20, 162}, {7, 4, 170}, {5, 18, 179},
	{6, 8, 189}, {5, 16, 201}, {7, 3, 213}, {4, 30, 220},
	{5, 14, 227}, {4, 28, 235}, {5, 13, 244}, {4, 26, 253},
	{5, 12, 262}, {4, 24, 273}, {7, 2, 284}, {4, 22, 297},
	{5, 10, 310}, {4, 20, 325}, {5, 9, 341}, {4, 18, 359},
	{5, 8, 380}, {4, 16, 402}, {7, 1, 427}, {3, 30, 441},
	{4, 14, 456}, {3, 28, 471}, {5, 6, 488}, {3, 26, 506},
	{4, 12, 525}, {3, 24, 546}, {4, 11, 569}, {3, 22, 594},
	{4, 10, 621}, {3, 20, 651}, {5, 4, 683}, {3, 18, 719},
	{3, 17, 759}, {3, 16, 804}, {2, 31, 854}, {2, 30, 882},
	{3, 14, 911}, {2, 28, 942}, {3, 13, 976}, {2, 26, 1012},
	{2, 25, 1051}, {2, 24, 1093}, {5, 2, 1139}, {2, 22, 1188},
	{3, 10, 1242}, {2, 20, 1302}, {4, 4, 1367}, {2, 18, 1439},
	{3, 8, 1519}, {2, 16, 1608}, {6, 0, 1708}, {1, 30, 1764},
	{2, 14, 1822}, {1, 28, 1885}, {3, 6, 1953}, {1, 26, 2025},
	{2, 12, 2103}, {1, 24, 2187}, {4, 2, 2278}, {1, 22, 2377},
	{2, 10, 2485}, {1, 20, 2604}, {3, 4, 2734}, {1, 18, 2878},
	{2, 8, 3036}, {1, 16, 3216}, {5, 0, 3417}, {0, 30, 3528},
	{1, 14, 3645}, {0, 28, 3771}, {2, 6, 3906}, {0, 26, 4050},
	{1, 12, 4206}, {0, 24, 4375}, {3, 2, 4557}, {0, 22, 4755},
	{1, 10, 4971}, {0, 20, 5208}, {2, 4, 5468}, {0, 18, 5753},
	{1, 8, 6067}, {0, 16, 6417}, {4, 0, 6803}, {0, 14, 7228},
	{1, 6, 7708}, {0, 12, 8445}, {2, 2, 9114}, {0, 10, 9943},
	{1, 4, 10937}, {0, 8, 12152}, {3, 0, 13671}, {0, 6, 15625},
	{1, 2, 18229}, {1, 4, 10937}, {0, 8, 12152}, {3, 0, 13671},
	{0, 6, 15625}, {1, 2, 18229}, {0, 4, 21875}, {2, 0, 27343},
	{0, 2, 36458}, {1, 0, 54687}
};

/* Motor state structure */
struct motor_state {
	int x_pos;              /* Current X position in steps */
	int y_pos;              /* Current Y position in steps */
	int x_max;              /* Maximum X steps */
	int y_max;              /* Maximum Y steps */
	int x_center;           /* Logical center for X axis */
	int y_center;           /* Logical center for Y axis */
	int x_ch;               /* Hardware channel for X-axis */
	int y_ch;               /* Hardware channel for Y-axis */
	int speed;              /* Current speed setting */
	u8 speed_reg_x;         /* Register value for X axis speed */
	u8 speed_reg_y;         /* Register value for Y axis speed */
	int target_x;           /* Pending X target when move completes */
	int target_y;           /* Pending Y target when move completes */
	unsigned long move_deadline_jiffies; /* Runaway cutoff timestamp */
	bool running;           /* Motor running flag */
	bool homed;             /* Has homing been completed? */
};

/* Module parameters */
static int hmaxstep = 7850;
static int vmaxstep = 2730;
static int x_channel = 1;
static int y_channel = 0;
static int max_speed = 1000;
static int home_position_center = 1;
static int center_x_override = -1;
static int center_y_override = -1;
static int invert_x = 0;
static int invert_y = 0;
static int move_timeout_ms = 15000;
module_param(hmaxstep, int, 0644);
MODULE_PARM_DESC(hmaxstep, "Maximum horizontal (X-axis) steps");
module_param(vmaxstep, int, 0644);
MODULE_PARM_DESC(vmaxstep, "Maximum vertical (Y-axis) steps");
module_param(x_channel, int, 0444);
MODULE_PARM_DESC(x_channel, "Hardware channel for X-axis (0 or 1, default 1)");
module_param(y_channel, int, 0444);
MODULE_PARM_DESC(y_channel, "Hardware channel for Y-axis (0 or 1, default 0)");
module_param(max_speed, int, 0644);
MODULE_PARM_DESC(max_speed, "Maximum motor speed (default 430)");
module_param(home_position_center, int, 0644);
MODULE_PARM_DESC(home_position_center, "Move to center position before setting home (1=yes, 0=no, default 1)");
module_param(center_x_override, int, 0644);
MODULE_PARM_DESC(center_x_override,
		 "Override X-axis logical center in absolute steps (-1 uses half range)");
module_param(center_y_override, int, 0644);
MODULE_PARM_DESC(center_y_override,
		 "Override Y-axis logical center in absolute steps (-1 uses half range)");
module_param(invert_x, int, 0644);
MODULE_PARM_DESC(invert_x, "Invert X-axis direction (1=inverted, 0=normal, default 0)");
module_param(invert_y, int, 0644);
MODULE_PARM_DESC(invert_y, "Invert Y-axis direction (1=inverted, 0=normal, default 0)");
module_param(move_timeout_ms, int, 0644);
MODULE_PARM_DESC(move_timeout_ms,
	       "Maximum milliseconds a move may run before the driver forces a stop (0 disables)");

#define HOMING_POLL_INTERVAL_MS 100
#define HOMING_PHASE_TIMEOUT_MS 60000

/* Module globals */
dev_t motor_dev_number;
static struct cdev motor_cdev;
static struct class *motor_class;
static struct task_struct *monitor_thread;
static struct motor_state motor;

/* Forward declarations */
static void motor_read_position(int *x_pos, int *y_pos);
static void motor_move_steps(int x_steps, int y_steps);
static void motor_stop(void);
static int wait_for_motor_idle(const char *phase, int timeout_ms);
static int homing_phase(int x_steps, int y_steps, const char *phase);
static void motor_refresh_center_targets(void);
static int motor_ops_goback(void);
static int motor_ops_reset(struct motor_reset_data *reset_data);

/**
 * Motor monitor thread - polls position every 100ms while motor is running
 */
static int motor_monitor_fn(void *data)
{
	int x_pos, y_pos;
	int prev_x_pos = 0, prev_y_pos = 0;
	int stable_count = 0;

	pr_debug("Motor monitor thread started\n");

	while (!kthread_should_stop()) {
		if (motor.running) {
			if (move_timeout_ms > 0 && motor.move_deadline_jiffies &&
			    time_after(jiffies, motor.move_deadline_jiffies)) {
				pr_err("Motor move exceeded %d ms, forcing stop\n",
					move_timeout_ms);
				motor_stop();
				motor.move_deadline_jiffies = 0;
				motor.homed = false;
				continue;
			}
			motor_read_position(&x_pos, &y_pos);

			/* Check if position has stopped changing */
			if (x_pos == prev_x_pos && y_pos == prev_y_pos) {
				stable_count++;
				/* Consider motor stopped after 2 consecutive stable readings (200ms) */
				if (stable_count >= 2) {
					/* Commit pending targets when hardware reports idle */
					motor.x_pos = motor.target_x;
					motor.y_pos = motor.target_y;
					motor.target_x = motor.x_pos;
					motor.target_y = motor.y_pos;
					motor.move_deadline_jiffies = 0;
					pr_info("Motor stopped: x=%d, y=%d\n",
						motor.x_pos, motor.y_pos);
					motor.running = false;
					stable_count = 0;
				}
			} else {
				stable_count = 0;
			}

			prev_x_pos = x_pos;
			prev_y_pos = y_pos;
		} else {
			/* Reset when motor not running */
			stable_count = 0;
		}
		msleep(100);
	}

	pr_debug("Motor monitor thread stopped\n");
	return 0;
}

/**
 * Homing function - moves motors to home position
 */
static int motor_homing(void)
{
	int half_x = motor.x_max > 0 ? motor.x_max / 2 : 0;
	int half_y = motor.y_max > 0 ? motor.y_max / 2 : 0;
	int center_x_target;
	int center_y_target;
	int ret;

	if (motor.x_max <= 0 && motor.y_max <= 0) {
		pr_err("Homing aborted - max step configuration missing\n");
		return -EINVAL;
	}

	motor_refresh_center_targets();
	center_x_target = motor.x_center;
	center_y_target = motor.y_center;

	pr_info("Starting enhanced homing sweep\n");

	ret = homing_phase(-half_x, -half_y, "preload sweep");
	if (ret)
		return ret;

	ret = homing_phase(motor.x_max, motor.y_max, "limit sweep");
	if (ret)
		return ret;

	/* Driver now knows we're sitting at the far limits */
	if (motor.x_max > 0)
		motor.x_pos = motor.x_max;
	else
		motor.x_pos = 0;
	if (motor.y_max > 0)
		motor.y_pos = motor.y_max;
	else
		motor.y_pos = 0;
	motor.target_x = motor.x_pos;
	motor.target_y = motor.y_pos;

	if (home_position_center) {
		int delta_x = 0;
		int delta_y = 0;

		if (motor.x_max > 0)
			delta_x = center_x_target - motor.x_pos;
		if (motor.y_max > 0)
			delta_y = center_y_target - motor.y_pos;

		ret = homing_phase(delta_x, delta_y, "center return");
		if (ret)
			return ret;
		motor.x_pos = center_x_target;
		motor.y_pos = center_y_target;
		motor.target_x = motor.x_pos;
		motor.target_y = motor.y_pos;
		pr_info("Enhanced homing complete - logical center (%d, %d)\n",
			motor.x_pos, motor.y_pos);
	} else {
		motor.x_pos = motor.x_max;
		motor.y_pos = motor.y_max;
		motor.target_x = motor.x_pos;
		motor.target_y = motor.y_pos;
		pr_info("Enhanced homing complete - origin left at (%d, %d)\n",
			motor.x_pos, motor.y_pos);
	}

	motor.homed = true;
	return 0;
}

/**
 * Read current motor position from chip registers
 */
static void motor_read_position(int *x_pos, int *y_pos)
{
	int pos_low, pos_high;
	unsigned int raw_value;
	int steps;
	int ret;

	mutex_lock(&tmi8152_spi_lock);

	/* Read X-axis position */
	ret = tmi8152_spi_read(ch_pos_l[motor.x_ch], &pos_low);
	if (ret < 0) {
		pr_err("Failed to read X-axis low position\n");
		mutex_unlock(&tmi8152_spi_lock);
		return;
	}
	ret = tmi8152_spi_read(ch_pos_h[motor.x_ch], &pos_high);
	if (ret < 0) {
		pr_err("Failed to read X-axis high position\n");
		mutex_unlock(&tmi8152_spi_lock);
		return;
	}
	raw_value = ((pos_high & 0x1F) << 8) | (pos_low & 0xFF);

	/* Decode position (13-bit bidirectional encoding) */
	if (raw_value < 0x1001) {
		steps = raw_value * TMI8152_POS_SCALE;
	} else {
		steps = -(TMI8152_MAX_POS - raw_value) * TMI8152_POS_SCALE;
	}
	*x_pos = steps;

	pr_debug("X-axis (ch%d): raw=0x%03X, steps=%d\n", motor.x_ch, raw_value, steps);

	/* Read Y-axis position */
	ret = tmi8152_spi_read(ch_pos_l[motor.y_ch], &pos_low);
	if (ret < 0) {
		pr_err("Failed to read Y-axis low position\n");
		mutex_unlock(&tmi8152_spi_lock);
		return;
	}
	ret = tmi8152_spi_read(ch_pos_h[motor.y_ch], &pos_high);
	if (ret < 0) {
		pr_err("Failed to read Y-axis high position\n");
		mutex_unlock(&tmi8152_spi_lock);
		return;
	}
	raw_value = ((pos_high & 0x1F) << 8) | (pos_low & 0xFF);

	/* Decode position (13-bit bidirectional encoding) */
	if (raw_value < 0x1001) {
		steps = raw_value * TMI8152_POS_SCALE;
	} else {
		steps = -(TMI8152_MAX_POS - raw_value) * TMI8152_POS_SCALE;
	}
	*y_pos = steps;

	pr_debug("Y-axis (ch%d): raw=0x%03X, steps=%d\n", motor.y_ch, raw_value, steps);

	mutex_unlock(&tmi8152_spi_lock);
}

/**
 * Motor movement function
 */
static void motor_move_steps(int x_steps, int y_steps)
{
	unsigned int target;
	unsigned int encoded_value;
	u8 target_low, target_high;
	u8 direction;
	bool movement = false;
	long pending;

	pr_debug("motor_move_steps: x=%d, y=%d, speed_x=0x%02x, speed_y=0x%02x\n",
		x_steps, y_steps, motor.speed_reg_x, motor.speed_reg_y);

	if (x_steps == 0 && y_steps == 0)
		return;

	/* Predict final logical coordinates so status matches daemon expectations */
	motor.target_x = motor.x_pos;
	motor.target_y = motor.y_pos;
	if (x_steps != 0) {
		pending = (long)motor.x_pos + x_steps;
		if (motor.x_max > 0)
			motor.target_x = clamp_t(int, pending, 0, motor.x_max);
		else
			motor.target_x = pending;
		movement = true;
	}
	if (y_steps != 0) {
		pending = (long)motor.y_pos + y_steps;
		if (motor.y_max > 0)
			motor.target_y = clamp_t(int, pending, 0, motor.y_max);
		else
			motor.target_y = pending;
		movement = true;
	}

	if (!movement)
		return;

	/* Clear both axis phase registers to 0 so inactive axis reads 0 */
	mutex_lock(&tmi8152_spi_lock);
	tmi8152_spi_write(ch_phase_l[motor.x_ch] | 0x80, 0x00);
	tmi8152_spi_write(ch_phase_h[motor.x_ch] | 0x80, 0x00);
	tmi8152_spi_write(ch_phase_l[motor.y_ch] | 0x80, 0x00);
	tmi8152_spi_write(ch_phase_h[motor.y_ch] | 0x80, 0x00);
	mutex_unlock(&tmi8152_spi_lock);

	/* Set motor running flag to start monitoring */
	motor.running = true;
	if (move_timeout_ms > 0)
		motor.move_deadline_jiffies = jiffies +
			msecs_to_jiffies(move_timeout_ms);
	else
		motor.move_deadline_jiffies = 0;

	/* Handle X-axis movement */
	if (x_steps != 0) {
		/* Calculate target position (steps / 16) */
		target = abs(x_steps) / TMI8152_POS_SCALE;

		/* Determine direction and encode target */
		if (x_steps > 0) {
			/* Move left - direct encoding */
			direction = invert_x ? TMI8152_DIR_RIGHT : TMI8152_DIR_LEFT;
			encoded_value = invert_x ? (TMI8152_MAX_POS - target) : target;
		} else {
			/* Move right - complemented encoding */
			direction = invert_x ? TMI8152_DIR_LEFT : TMI8152_DIR_RIGHT;
			encoded_value = invert_x ? target : (TMI8152_MAX_POS - target);
		}

		target_low = encoded_value & 0xFF;
		target_high = (encoded_value >> 8) & 0x1F;

		pr_debug("X-axis (ch%d): steps=%d, target=0x%03X, encoded=0x%03X, dir=0x%02X\n",
			motor.x_ch, x_steps, target, encoded_value, direction);

		/* Execute movement sequence */
		mutex_lock(&tmi8152_spi_lock);
		tmi8152_spi_write(ch_ctrl[motor.x_ch] | 0x80, TMI8152_MODE_POS);
		tmi8152_spi_write(ch_dir[motor.x_ch] | 0x80, direction);
		tmi8152_spi_write(ch_speed[motor.x_ch] | 0x80, motor.speed_reg_x);
		tmi8152_spi_write(ch_ctrl[motor.x_ch] | 0x80, TMI8152_MODE_POS);
		tmi8152_spi_write(ch_tgt_l[motor.x_ch] | 0x80, target_low);
		tmi8152_spi_write(ch_tgt_h[motor.x_ch] | 0x80, target_high);
		tmi8152_spi_write(ch_ctrl[motor.x_ch] | 0x80,
				  TMI8152_MODE_POS | TMI8152_START);
		mutex_unlock(&tmi8152_spi_lock);
	}

	/* Handle Y-axis movement */
	if (y_steps != 0) {
		/* Calculate target position (steps / 16) */
		target = abs(y_steps) / TMI8152_POS_SCALE;

		/* Determine direction and encode target */
		if (y_steps > 0) {
			/* Move up - direct encoding */
			direction = invert_y ? TMI8152_DIR_RIGHT : TMI8152_DIR_LEFT;
			encoded_value = invert_y ? (TMI8152_MAX_POS - target) : target;
		} else {
			/* Move down - complemented encoding */
			direction = invert_y ? TMI8152_DIR_LEFT : TMI8152_DIR_RIGHT;
			encoded_value = invert_y ? target : (TMI8152_MAX_POS - target);
		}

		target_low = encoded_value & 0xFF;
		target_high = (encoded_value >> 8) & 0x1F;

		pr_debug("Y-axis (ch%d): steps=%d, target=0x%03X, encoded=0x%03X, dir=0x%02X\n",
			motor.y_ch, y_steps, target, encoded_value, direction);

		/* Execute movement sequence */
		mutex_lock(&tmi8152_spi_lock);
		tmi8152_spi_write(ch_ctrl[motor.y_ch] | 0x80, TMI8152_MODE_POS);
		tmi8152_spi_write(ch_dir[motor.y_ch] | 0x80, direction);
		tmi8152_spi_write(ch_speed[motor.y_ch] | 0x80, motor.speed_reg_y);
		tmi8152_spi_write(ch_ctrl[motor.y_ch] | 0x80, TMI8152_MODE_POS);
		tmi8152_spi_write(ch_tgt_l[motor.y_ch] | 0x80, target_low);
		tmi8152_spi_write(ch_tgt_h[motor.y_ch] | 0x80, target_high);
		tmi8152_spi_write(ch_ctrl[motor.y_ch] | 0x80,
				  TMI8152_MODE_POS | TMI8152_START);
		mutex_unlock(&tmi8152_spi_lock);
	}

	pr_debug("Motor movement sequence completed\n");
}

static int wait_for_motor_idle(const char *phase, int timeout_ms)
{
	int loops;

	if (timeout_ms <= 0)
		timeout_ms = HOMING_PHASE_TIMEOUT_MS;

	loops = DIV_ROUND_UP(timeout_ms, HOMING_POLL_INTERVAL_MS);
	while (motor.running && loops-- > 0)
		msleep(HOMING_POLL_INTERVAL_MS);

	if (motor.running) {
		pr_err("Homing timeout while %s\n", phase);
		motor.running = false;
		return -ETIMEDOUT;
	}

	pr_debug("Homing %s completed\n", phase);
	return 0;
}

static int homing_phase(int x_steps, int y_steps, const char *phase)
{
	if (x_steps == 0 && y_steps == 0)
		return 0;

	pr_debug("Homing phase %s: x=%d, y=%d\n", phase, x_steps, y_steps);
	motor_move_steps(x_steps, y_steps);
	return wait_for_motor_idle(phase, HOMING_PHASE_TIMEOUT_MS);
}

static void motor_refresh_center_targets(void)
{
	if (motor.x_max > 0) {
		if (center_x_override >= 0)
			motor.x_center = clamp_t(int, center_x_override, 0, motor.x_max);
		else
			motor.x_center = motor.x_max / 2;
	} else {
		motor.x_center = 0;
	}

	if (motor.y_max > 0) {
		if (center_y_override >= 0)
			motor.y_center = clamp_t(int, center_y_override, 0, motor.y_max);
		else
			motor.y_center = motor.y_max / 2;
	} else {
		motor.y_center = 0;
	}
}

static int motor_ops_goback(void)
{
	int delta_x;
	int delta_y;

	if (!motor.homed) {
		pr_warn("MOTOR_GOBACK requested before homing complete\n");
		return -EINVAL;
	}

	delta_x = motor.x_center - motor.x_pos;
	delta_y = motor.y_center - motor.y_pos;

	if (delta_x == 0 && delta_y == 0) {
		pr_debug("Already at logical center, no goback needed\n");
		return 0;
	}

	pr_debug("Returning to logical center: dx=%d, dy=%d\n", delta_x, delta_y);
	motor_move_steps(delta_x, delta_y);
	return 0;
}

static int motor_ops_reset(struct motor_reset_data *reset_data)
{
	int ret;

	if (reset_data && reset_data->x_max_steps && reset_data->y_max_steps) {
		motor.x_max = reset_data->x_max_steps;
		motor.y_max = reset_data->y_max_steps;
		motor_refresh_center_targets();
		motor.x_pos = clamp_t(int, (int)reset_data->x_cur_step, 0, motor.x_max);
		motor.y_pos = clamp_t(int, (int)reset_data->y_cur_step, 0, motor.y_max);
		motor.target_x = motor.x_pos;
		motor.target_y = motor.y_pos;
		motor.homed = true;
		return 0;
	}

	ret = motor_homing();
	if (ret)
		return ret;

	if (reset_data) {
		reset_data->x_max_steps = motor.x_max;
		reset_data->y_max_steps = motor.y_max;
		reset_data->x_cur_step = motor.x_pos;
		reset_data->y_cur_step = motor.y_pos;
	}

	return 0;
}

/**
 * Binary search to find speed table index for given speed value
 */
static int binary_search_speed_table(u32 target_value)
{
	int low = 0;
	int high = 141;  /* 142 entries, indices 0-141 */
	int mid;

	while (low <= high) {
		mid = (low + high + 1) / 2;

		if (low == high)
			return high;

		if (target_value < speed_profile_table[mid].threshold) {
			high = mid - 1;
		} else if (target_value == speed_profile_table[mid].threshold) {
			return mid;
		} else {
			low = mid;
		}
	}

	return high;
}

static int clamp_speed_value(int speed_value)
{
	if (speed_value > max_speed) {
		pr_debug("Speed %d exceeds max_speed %d, clamping\n",
			speed_value, max_speed);
		speed_value = max_speed;
	}
	if (speed_value < 0) {
		pr_warn("Negative speed %d, clamping to 0\n", speed_value);
		speed_value = 0;
	}

	return speed_value;
}

static u8 speed_value_to_reg(int speed_value)
{
	int index;
	u8 multiplier, base;

	index = binary_search_speed_table(speed_value);
	multiplier = speed_profile_table[index].multiplier;
	base = speed_profile_table[index].base;

	return base + (multiplier * 32);
}

/**
 * Stop motor movement
 */
static void motor_stop(void)
{
	int x_pos, y_pos;

	pr_debug("Stopping motors\n");

	/* Stop X-axis motor */
	mutex_lock(&tmi8152_spi_lock);
	tmi8152_spi_write(ch_ctrl[motor.x_ch] | 0x80, 0x00);

	/* Stop Y-axis motor */
	tmi8152_spi_write(ch_ctrl[motor.y_ch] | 0x80, 0x00);
	mutex_unlock(&tmi8152_spi_lock);

	/* Read current chip position */
	motor_read_position(&x_pos, &y_pos);

	/* Update position based on actual chip movement */
	motor.x_pos += x_pos;
	motor.y_pos += y_pos;
	if (motor.x_max > 0) {
		if (motor.x_pos < 0)
			motor.x_pos = 0;
		else if (motor.x_pos > motor.x_max)
			motor.x_pos = motor.x_max;
	}
	if (motor.y_max > 0) {
		if (motor.y_pos < 0)
			motor.y_pos = 0;
		else if (motor.y_pos > motor.y_max)
			motor.y_pos = motor.y_max;
	}

	/* Clear running flag */
	motor.running = false;
	motor.move_deadline_jiffies = 0;
	motor.target_x = motor.x_pos;
	motor.target_y = motor.y_pos;

	pr_debug("Motors stopped at position: x=%d, y=%d\n",
		motor.x_pos, motor.y_pos);
}

/**
 * Device file operations
 */
static int motor_open(struct inode *inode, struct file *file)
{
	pr_debug("Motor device opened\n");
	return 0;
}

static int motor_release(struct inode *inode, struct file *file)
{
	pr_debug("Motor device released\n");
	return 0;
}

static long motor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct motors_steps steps;
	struct motor_message msg;
	int ret;

	switch (cmd) {
	case MOTOR_STOP:
		pr_debug("MOTOR_STOP\n");
		motor_stop();
		break;

	case MOTOR_MOVE:
		/* Copy movement data from userspace */
		if (copy_from_user(&steps, (void __user *)arg, sizeof(steps))) {
			pr_err("Failed to copy motor steps from userspace\n");
			return -EFAULT;
		}

		pr_debug("MOTOR_MOVE: x=%d, y=%d\n", steps.x, steps.y);

		/* Execute motor movement */
		motor_move_steps(steps.x, steps.y);

		break;

	case MOTOR_GET_STATUS:
		pr_debug("MOTOR_GET_STATUS\n");

		/* Populate status message */
		memset(&msg, 0, sizeof(msg));
		msg.x = motor.x_pos;
		msg.y = motor.y_pos;
		msg.status = motor.running ? MOTOR_IS_RUNNING : MOTOR_IS_STOP;
		msg.speed = motor.speed;
		msg.x_max_steps = motor.x_max;
		msg.y_max_steps = motor.y_max;
		msg.inversion_state = 0;  /* Not implemented yet */

		/* Copy status to userspace */
		if (copy_to_user((void __user *)arg, &msg, sizeof(msg))) {
			pr_err("Failed to copy status to userspace\n");
			return -EFAULT;
		}

		break;

	case MOTOR_GOBACK:
		ret = motor_ops_goback();
		if (ret)
			return ret;
		break;

	case MOTOR_SPEED:
		{
			int speed_value;
			u8 reg;

			if (copy_from_user(&speed_value, (void __user *)arg, sizeof(int))) {
				pr_err("Failed to copy speed from userspace\n");
				return -EFAULT;
			}

			speed_value = clamp_speed_value(speed_value);
			reg = speed_value_to_reg(speed_value);
			motor.speed_reg_x = reg;
			motor.speed_reg_y = reg;
			motor.speed = reg;

			pr_debug("MOTOR_SPEED: input=%d, register=0x%02x\n",
				speed_value, reg);
		}
		break;

	case MOTOR_SPEED_AXIS:
		{
			struct motor_axis_speed axis_speed;
			u8 reg_x;
			u8 reg_y;
			int clamped_x;
			int clamped_y;

			if (copy_from_user(&axis_speed, (void __user *)arg,
					sizeof(axis_speed))) {
				pr_err("Failed to copy axis speeds from userspace\n");
				return -EFAULT;
			}

			clamped_x = clamp_speed_value(axis_speed.x_speed);
			clamped_y = clamp_speed_value(axis_speed.y_speed);
			reg_x = speed_value_to_reg(clamped_x);
			reg_y = speed_value_to_reg(clamped_y);

			motor.speed_reg_x = reg_x;
			motor.speed_reg_y = reg_y;
			motor.speed = max_t(int, reg_x, reg_y);

			pr_debug("MOTOR_SPEED_AXIS: input_x=%d (reg=0x%02x), input_y=%d (reg=0x%02x)\n",
				clamped_x, reg_x, clamped_y, reg_y);
		}
		break;

	case MOTOR_RESET:
		{
			struct motor_reset_data reset_data;

			pr_debug("MOTOR_RESET ioctl received\n");

			if (copy_from_user(&reset_data, (void __user *)arg, sizeof(reset_data))) {
				pr_err("Failed to copy reset data from userspace\n");
				return -EFAULT;
			}

			ret = motor_ops_reset(&reset_data);
			if (ret)
				return ret;

			if (copy_to_user((void __user *)arg, &reset_data, sizeof(reset_data))) {
				pr_err("Failed to copy reset data back to userspace\n");
				return -EFAULT;
			}
		}
		break;

	default:
		pr_err("Unknown ioctl command: 0x%x\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static struct file_operations motor_fops = {
	.owner = THIS_MODULE,
	.open = motor_open,
	.release = motor_release,
	.unlocked_ioctl = motor_ioctl,
};

/**
 * Module initialization
 */
static int __init motor_init(void)
{
	struct device *dev;
	int ret;

	pr_info("Initializing simple TMI8152 motor driver\n");

	/* Validate channel configuration */
	if (x_channel < 0 || x_channel > 1) {
		pr_err("Invalid x_channel=%d (must be 0 or 1)\n", x_channel);
		return -EINVAL;
	}
	if (y_channel < 0 || y_channel > 1) {
		pr_err("Invalid y_channel=%d (must be 0 or 1)\n", y_channel);
		return -EINVAL;
	}
	if (x_channel == y_channel) {
		pr_err("x_channel and y_channel must be different (both set to %d)\n", x_channel);
		return -EINVAL;
	}

	/* Initialize motor state */
	memset(&motor, 0, sizeof(motor));
	motor.x_ch = x_channel;
	motor.y_ch = y_channel;
	motor.x_max = hmaxstep;
	motor.y_max = vmaxstep;
	motor_refresh_center_targets();
	motor.x_pos = motor.x_center;
	motor.y_pos = motor.y_center;
	motor.target_x = motor.x_pos;
	motor.target_y = motor.y_pos;
	motor.move_deadline_jiffies = 0;

	/* Default to 95% of max_speed using speed profile table */
	{
		int index;
		u32 speed_95pct = (max_speed * 95) / 100;
		u8 multiplier, base;

		index = binary_search_speed_table(speed_95pct);
		multiplier = speed_profile_table[index].multiplier;
		base = speed_profile_table[index].base;
		motor.speed = base + (multiplier * 32);
		motor.speed_reg_x = motor.speed;
		motor.speed_reg_y = motor.speed;

		pr_debug("Default speed: 95%% (%d/%d) -> index=%d, register=0x%02x\n",
			speed_95pct, max_speed, index, motor.speed);
	}

	motor.running = false;
	motor.homed = true;

	pr_debug("Channel mapping: X-axis -> Channel %d, Y-axis -> Channel %d\n", motor.x_ch, motor.y_ch);

	/* Allocate character device region */
	ret = alloc_chrdev_region(&motor_dev_number, 0, 1, "motor");
	if (ret < 0) {
		pr_err("Failed to allocate chrdev region: %d\n", ret);
		return ret;
	}

	/* Initialize character device */
	cdev_init(&motor_cdev, &motor_fops);
	motor_cdev.owner = THIS_MODULE;

	ret = cdev_add(&motor_cdev, motor_dev_number, 1);
	if (ret < 0) {
		pr_err("Failed to add cdev: %d\n", ret);
		goto err_cdev_add;
	}

	/* Create device class */
	motor_class = class_create(THIS_MODULE, "motor");
	if (IS_ERR(motor_class)) {
		pr_err("Failed to create class\n");
		ret = PTR_ERR(motor_class);
		goto err_class;
	}

	/* Create device node */
	dev = device_create(motor_class, NULL, motor_dev_number, NULL, "motor");
	if (IS_ERR(dev)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(dev);
		goto err_device;
	}

	mutex_lock(&tmi8152_spi_lock);
	tmi8152_spi_write(0x82, 0x02);
	tmi8152_spi_write(0x82, 0x8B);
	tmi8152_spi_write(0x8a, 0x00);
	tmi8152_spi_write(0x8F, 0x00);
	tmi8152_spi_write(0x9A, 0x00);
	tmi8152_spi_write(0x92, 0x00);
	mutex_unlock(&tmi8152_spi_lock);

	/* Start motor monitor thread */
	monitor_thread = kthread_run(motor_monitor_fn, NULL, "motor_monitor");
	if (IS_ERR(monitor_thread)) {
		pr_err("Failed to create monitor thread\n");
		ret = PTR_ERR(monitor_thread);
		goto err_thread;
	}

	pr_info("Simple TMI8152 motor driver initialized successfully\n");
	pr_info("Waiting for MOTOR_RESET to perform homing sequence\n");
	return 0;

err_thread:
	device_destroy(motor_class, motor_dev_number);

err_device:
	class_destroy(motor_class);
err_class:
	cdev_del(&motor_cdev);
err_cdev_add:
	unregister_chrdev_region(motor_dev_number, 1);
	return ret;
}

/**
 * Module cleanup
 */
static void __exit motor_exit(void)
{
	pr_info("Removing simple TMI8152 motor driver\n");

	/* Stop motor monitor thread */
	if (monitor_thread) {
		kthread_stop(monitor_thread);
	}

	device_destroy(motor_class, motor_dev_number);
	class_destroy(motor_class);
	cdev_del(&motor_cdev);
	unregister_chrdev_region(motor_dev_number, 1);
}

module_init(motor_init);
module_exit(motor_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("sstepansky");
MODULE_DESCRIPTION("Simple TMI8152 motor driver");
