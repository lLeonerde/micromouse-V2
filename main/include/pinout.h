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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "esp_system.h"

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
#define I2C_MASTER_FREQ_HZ   50000      // Frequency do I2C


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

/*
#define KP 0.070     // Constante Proporcional
#define KI 0.0015// Constante Integral 
#define KD 0.677 // Constante Derivativa*/

#define KP 0.065     // Constante Proporcional
#define KI 0.002// Constante Integral 
#define KD 0.600 // Constante Derivativa
#define MAX_SPEED 100  // Velocidade máxima em %
#define MIN_SPEED -100    // Velocidade mínima em %
#define TARGET_ENCODER_VALUE1 -470  // Valor-alvo do encoder 820 step
#define TARGET_ENCODER_VALUE2 470  // Valor-alvo do encoder
#define ERROR_TOLERANCE 2         
#define MIN_DUTY_CYCLE 40

//#define MOTOR_OFSET 100


#define MAZE_SIZE 8  // Define o tamanho do labirinto como 8x8 células
#define MAX_PATH_LENGTH (MAZE_SIZE * MAZE_SIZE)  // Máximo de células que podem fazer parte do caminho
#define INF 99999  // Define um valor de "infinito" para inicializar as distâncias

// Estrutura que armazena coordenadas (x, y) e a direção do robô
typedef struct {
    int x;  // Posição no eixo X
    int y;  // Posição no eixo Y
    int direction;  // Direção atual do robô: 0=Norte, 1=Leste, 2=Sul, 3=Oeste
} Coord;

// Estrutura que representa cada célula do labirinto
typedef struct {
    bool visited;  // Indica se a célula já foi visitada
    bool wallNorth;  // Indica se há uma parede ao norte
    bool wallSouth;  // Indica se há uma parede ao sul
    bool wallEast;   // Indica se há uma parede a leste
    bool wallWest;   // Indica se há uma parede a oeste
    int distance;    // Distância da célula até o alvo, usada para o cálculo do caminho
} Cell;

// Estrutura de um nó para a fila encadeada, usada no BFS
typedef struct Node {
    Coord coord;  // Coordenada do nó
    struct Node *next;  // Próximo nó na fila
} Node;

// Estrutura da fila encadeada
typedef struct {
    Node *front;  // Posição inicial da fila
    Node *rear;   // Posição final da fila
    int size;     // Tamanho atual da fila
} Queue;

// Estrutura que armazena o estado do labirinto e do robô
typedef struct {
    Cell maze[MAZE_SIZE][MAZE_SIZE];  // Matriz representando o labirinto
    Coord path[MAX_PATH_LENGTH];      // Array que armazena o caminho percorrido
    int pathIndex;                    // Índice do caminho atual
    Coord current;                    // Posição e direção atual do robô
    bool targetReached;               // Indica se o robô chegou ao alvo (centro do labirinto)
} MazeState;




#endif