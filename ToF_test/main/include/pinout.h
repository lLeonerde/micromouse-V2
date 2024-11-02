#ifndef PINOUT_H_
#define PINOUT_H_
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "esp_task_wdt.h"

//local libraries
#include "vl53l0x.h"
//motor define
#define MotFwd_1  25  // Motor 1 Forward pin
#define MotRev_1  26  // Motor 1 Reverse pin

#define MotFwd_2  32 // Motor 2 Forward pin
#define MotRev_2  33 // Motor 2 Reverse pin

#define encoderPin1_1  35 // Encoder 1 Output 'A'
#define encoderPin2_1  34 // Encoder 1 Output 'B'

#define encoderPin1_2  2 // Encoder 2 Output 'A'
#define encoderPin2_2  14 // Encoder 2 Output 'B'


//ToF xshut define
#define XSHUT_PIN_SENSOR_1   15 
#define XSHUT_PIN_SENSOR_2   4
#define XSHUT_PIN_SENSOR_3   23
#define XSHUT_PIN_SENSOR_4   18
#define XSHUT_PIN_SENSOR_5   19

//I2C parameters
#define I2C_MASTER_SCL_IO    21    // SCL
#define I2C_MASTER_SDA_IO    22    // SDA
#define I2C_HOST      I2C_NUM_0   // I2C port
#define I2C_MASTER_FREQ_HZ   300000      // Frequency do I2C

#define MOTOR_OFSET 100

#endif