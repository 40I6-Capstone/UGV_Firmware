/**
 * @file hwdefs.h
 * @author Shaqeeb Momen
 * @brief Definitions for hardware connections
 * @version 0.1
 * @date 2022-11-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <hardware/i2c.h>

#define PIN_UART0_TX 0
#define PIN_UART0_RX 1

#define PIN_MOTOR_RA 13
#define PIN_MOTOR_RB 12
#define PIN_MOTOR_LA 10
#define PIN_MOTOR_LB 11

#define PIN_SDA 4
#define PIN_SCL 5
#define I2C_INST i2c0

#define PIN_IMU_INT1 8
#define PIN_IMU_INT2 9

#define PIN_ENC_RA 16
#define PIN_ENC_RB 17
#define PIN_ENC_LA 14
#define PIN_ENC_LB 15

#define PIN_LED_R 20
#define PIN_LED_G 19
#define PIN_LED_B 18