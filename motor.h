#ifndef _MOTOR_H_
#define _MOTOR_H_

/* IOCTL Command Definitions */
#define MOTOR_STOP                  0x1
#define MOTOR_RESET                 0x2
#define MOTOR_MOVE                  0x3
#define MOTOR_GET_STATUS            0x4
#define MOTOR_SPEED                 0x5
#define MOTOR_GOBACK                0x6
#define MOTOR_CRUISE                0x7

/* Motor Status */
enum motor_status {
	MOTOR_IS_STOP,
	MOTOR_IS_RUNNING,
};

/* Data Structures for IOCTL Communication */

/**
 * struct motors_steps - Motor movement in steps
 * Used with MOTOR_MOVE ioctl
 */
struct motors_steps {
	int x;
	int y;
};

/**
 * struct motor_reset_data - Homing/reset configuration
 * Used with MOTOR_RESET ioctl
 */
struct motor_reset_data {
	unsigned int x_max_steps;
	unsigned int y_max_steps;
	unsigned int x_cur_step;
	unsigned int y_cur_step;
};

/**
 * struct motor_message - Motor status information
 * Used with MOTOR_GET_STATUS ioctl
 */
struct motor_message {
	int x;
	int y;
	enum motor_status status;
	int speed;
	unsigned int x_max_steps;
	unsigned int y_max_steps;
	unsigned int inversion_state;
};

#endif /* _MOTOR_H_ */
