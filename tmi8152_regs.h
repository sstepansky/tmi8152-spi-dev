/*
 * tmi8152_regs.h - TMI8152 Motor Controller Register Definitions
 *
 * Register map and constants for the TMI8152 dual-channel stepper motor
 * controller chip. This header is shared between the SPI driver and the
 * motor driver.
 *
 * All the register names are made up by me based on reverse engineering.
 */

#ifndef _TMI8152_REGS_H_
#define _TMI8152_REGS_H_

/*
 * Chip Identification Registers
 */
#define TMI8152_CHIP_ID_H           0x00  /* Chip ID high byte */
#define TMI8152_CHIP_ID_L           0x01  /* Chip ID low byte */

/*
 * Global Control/Status Register
 */
#define TMI8152_CTRL_STATUS         0x82  /* Main chip control/status register */

/*
 * Channel 1 Registers
 */
#define TMI8152_CH1_INIT            0x02  /* Initialization register */
#define TMI8152_CH1_CTRL            0x04  /* Control mode register */
#define TMI8152_CH1_DIR             0x05  /* Direction register */
#define TMI8152_CH1_SPEED           0x06  /* Speed control register */
#define TMI8152_CH1_POS_L           0x08  /* Position low byte */
#define TMI8152_CH1_POS_H           0x09  /* Position high byte */
#define TMI8152_CH1_PWR             0x0A  /* Power mode register */
#define TMI8152_CH1_TGT_L           0x0B  /* Target position low byte */
#define TMI8152_CH1_TGT_H           0x0C  /* Target position high byte */
#define TMI8152_CH1_STAT            0x0D  /* Status register */
#define TMI8152_CH1_PASS            0x0E  /* Pass register */
#define TMI8152_CH1_PHASE           0x0F  /* Phase register */

/*
 * Channel 2 Registers (0x10 offset from Channel 1)
 */
#define TMI8152_CH2_TQ_REL          0x12  /* Torque release register */
#define TMI8152_CH2_CTRL            0x14  /* Control mode register */
#define TMI8152_CH2_DIR             0x15  /* Direction register */
#define TMI8152_CH2_SPEED           0x16  /* Speed control register */
#define TMI8152_CH2_POS_L           0x18  /* Position low byte */
#define TMI8152_CH2_POS_H           0x19  /* Position high byte */
#define TMI8152_CH2_PWR             0x1A  /* Power mode register */
#define TMI8152_CH2_TGT_L           0x1B  /* Target position low byte */
#define TMI8152_CH2_TGT_H           0x1C  /* Target position high byte */
#define TMI8152_CH2_STAT            0x1D  /* Status register */
#define TMI8152_CH2_PASS            0x1E  /* Pass register */
#define TMI8152_CH2_TQ_AUX          0x1F  /* Torque auxiliary register */

/*
 * Phase Clear Registers
 * Note: These overlap with position registers but are used during initialization
 */
#define TMI8152_CH1_PHASE_L         0x08  /* Channel 1 phase clear low */
#define TMI8152_CH1_PHASE_H         0x09  /* Channel 1 phase clear high */
#define TMI8152_CH2_PHASE_L         0x18  /* Channel 2 phase clear low */
#define TMI8152_CH2_PHASE_H         0x19  /* Channel 2 phase clear high */

/*
 * IR-Cut Control Registers
 */
#define TMI8152_IR_CUT_CONTROL      0x11  /* IR cut filter control */

/*
 * Direction Values
 * Written to DIR registers (0x05, 0x15)
 */
#define TMI8152_DIR_LEFT            0x75  /* Channel 1 left / Channel 2 up */
#define TMI8152_DIR_RIGHT           0x7A  /* Channel 1 right / Channel 2 down */

/*
 * Control Mode Values
 * Written to CTRL registers (0x04, 0x14)
 */
#define TMI8152_MODE_POS            0x02  /* Position control mode - stops at target */
#define TMI8152_MODE_CONT           0x03  /* Continuous rotation mode */
#define TMI8152_MODE_HITORQ         0x07  /* High torque mode (unstable) */

/*
 * Motor Start Bit
 * OR with control mode to start motor movement
 * Example: write (TMI8152_MODE_POS | TMI8152_START) to CTRL register
 */
#define TMI8152_START               0x80

/*
 * Power Modes
 * Written to PWR registers (0x0A, 0x1A)
 */
#define TMI8152_PWR_NORM            0x00  /* Normal power mode */
#define TMI8152_PWR_LOW             0x80  /* Low power mode */
#define TMI8152_PWR_SPEC            0xFF  /* Special power mode */

/*
 * Position Encoding Constants
 */
#define TMI8152_POS_SCALE           16    /* All positions scaled by factor of 16 */
#define TMI8152_MAX_POS             0x2000 /* 13-bit max value (8192) */
#define TMI8152_STEPS_PER_ROT       8192  /* Steps for full 360° rotation */

/*
 * Register Access
 * Bit 7 (0x80) must be set for write operations
 * Example: tmi8152_spi_write(TMI8152_CH1_SPEED | 0x80, speed_value);
 */
#define TMI8152_WRITE_BIT           0x80  /* OR with register address for writes */

/*
 * Chip ID Expected Values
 */
#define TMI8152_EXPECTED_ID_H       0x81  /* Expected chip ID high byte */
#define TMI8152_EXPECTED_ID_L       0x50  /* Expected chip ID low byte */

/*
 * Control/Status Register Values (0x82)
 */
#define TMI8152_CTRL_INIT           0x03  /* Initialize chip */
#define TMI8152_CTRL_ENABLE         0x8B  /* Enable chip */
#define TMI8152_CTRL_DISABLE        0xC1  /* Disable chip */

/*
 * IR-Cut Command Values (0x91)
 */
#define TMI8152_IR_CUT_OFF          0x86  /* IR filter off */
#define TMI8152_IR_CUT_ON           0x8A  /* IR filter on */

#endif /* _TMI8152_REGS_H_ */
