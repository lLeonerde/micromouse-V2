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
#include "driver/ledc.h"


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


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO_FWD_1    MotFwd_1 // Pino PWM Motor 1 para frente
#define LEDC_OUTPUT_IO_REV_1    MotRev_1 // Pino PWM Motor 1 para trás
#define LEDC_OUTPUT_IO_FWD_2    MotFwd_2 // Pino PWM Motor 2 para frente
#define LEDC_OUTPUT_IO_REV_2    MotRev_2 // Pino PWM Motor 2 para trás
#define LEDC_CHANNEL_FWD_1      LEDC_CHANNEL_0
#define LEDC_CHANNEL_REV_1      LEDC_CHANNEL_1
#define LEDC_CHANNEL_FWD_2      LEDC_CHANNEL_2
#define LEDC_CHANNEL_REV_2      LEDC_CHANNEL_3
#define LEDC_FREQUENCY          5000     // Frequência PWM em Hz
#define LEDC_RESOLUTION         LEDC_TIMER_13_BIT

#define KP 0.15  // Constante Proporcional
#define KI 0.015 // Constante Integral
#define KD 0.001 // Constante Derivativa
#define MAX_SPEED 90  // Velocidade máxima em %
#define MIN_SPEED -90    // Velocidade mínima em %
#define TARGET_ENCODER_VALUE1 470  // Valor-alvo do encoder 820 step
#define TARGET_ENCODER_VALUE2 -470  // Valor-alvo do encoder
#define ERROR_TOLERANCE 2          
#define MIN_DUTY_CYCLE 30

//#define MOTOR_OFSET 100

#endif