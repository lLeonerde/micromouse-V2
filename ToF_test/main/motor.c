#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Definições de pinos
#define MotFwd_1  2  // Motor 1 Forward pin
#define MotRev_1  4  // Motor 1 Reverse pin

#define MotFwd_2  18 // Motor 2 Forward pin
#define MotRev_2  19 // Motor 2 Reverse pin

#define encoderPin1_1  13 // Encoder 1 Output 'A'
#define encoderPin2_1  12 // Encoder 1 Output 'B'

#define encoderPin1_2  27 // Encoder 2 Output 'A'
#define encoderPin2_2  26 // Encoder 2 Output 'B'

// Variáveis do encoder 1
volatile int lastEncoded_1 = 0;
volatile long encoderValue_1 = 0;

// Variáveis do encoder 2
volatile int lastEncoded_2 = 0;
volatile long encoderValue_2 = 0;

// Declaração das tasks
TaskHandle_t motorfrenttask = NULL;
TaskHandle_t motortrastask = NULL;
TaskHandle_t motorparartask = NULL;
TaskHandle_t motordireitatask = NULL;
TaskHandle_t motoresquerdatask = NULL;
TimerHandle_t xTimer = NULL; // Handle do timer

// Função de interrupção para o encoder 1
void IRAM_ATTR updateEncoder_1() {
    int MSB = gpio_get_level(encoderPin1_1);
    int LSB = gpio_get_level(encoderPin2_1);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded_1 << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_1--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_1++;

    lastEncoded_1 = encoded;
}

// Função de interrupção para o encoder 2
void IRAM_ATTR updateEncoder_2() {
    int MSB = gpio_get_level(encoderPin1_2);
    int LSB = gpio_get_level(encoderPin2_2);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded_2 << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_2--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_2++;

    lastEncoded_2 = encoded;
}

// Função de callback do timer
void vTimerCallback(TimerHandle_t xTimer) {
    static int direction = 0; // Alterna entre frente (0) e trás (1)

    // Talvez de para colocar o print dos valores dos enconder aqui a cada troca pois do jeito q ta nao e mto legivel ele imprimir a todo momento na task de movimento
    
    if (direction == 0) {
        vTaskSuspend(motorfrenttask); // Suspender a task de ir para frente
        vTaskResume(motorparartask); // Retoma a task de parar
        vTaskSuspend(motordireitatask); // Suspender a task de ir para direita
        vTaskSuspend(motoresquerdatask); // Suspender a task de ir para esquerda
        vTaskSuspend(motortrastask);   // Susperder a task de ir para trás
        direction = 1;                // Alternar para direita
    } else if (direction == 1){

        vTaskSuspend(motorfrenttask); // Suspender a task de ir para frente
        vTaskSuspend(motorparartask); // Suspender a task de parar
        vTaskResume(motordireitatask); // Retorna a task de ir para direita
        vTaskSuspend(motoresquerdatask); // Suspender a task de ir para esquerda
        vTaskSuspend(motortrastask);   // Suspender a task de ir para trás
        direction = 2;                // Alternar para esquerda
    } else if (direction == 2){

        vTaskSuspend(motorfrenttask); // Suspender a task de ir para frente
        vTaskSuspend(motorparartask); // Suspender a task de parar
        vTaskSuspend(motordireitatask); // Suspender a task de ir para direita
        vTaskResume(motoresquerdatask); // Retomar a task de ir para esquerda
        vTaskSuspend(motortrastask);   // Suspender a task de ir para trás
        direction = 3;                // Alternar para tras
    } else if (direction == 3){

        vTaskSuspend(motorfrenttask); // Suspender a task de ir para frente
        vTaskSuspend(motorparartask); // Suspender a task de parar
        vTaskSuspend(motordireitatask); // Suspender a task de ir para direita
        vTaskSuspend(motoresquerdatask); // Suspender a task de ir para esquerda
        vTaskResume(motortrastask);   // Retomar a task de ir para trás
        direction = 4;                // Alternar para frente
    } else if (direction == 4){

        vTaskResume(motorfrenttask); // RETOMA a task de ir para frente
        vTaskSuspend(motorparartask); // Suspender a task de parar
        vTaskSuspend(motordireitatask); // Suspender a task de ir para direita
        vTaskSuspend(motoresquerdatask); // Suspender a task de ir para esquerda
        vTaskSuspend(motortrastask);   // Suspender a task de ir para trás
        direction = 0;                // Alternar para parar
    }
}

// Task para ir para frente
void motorfrent(void *arg) {
    while (1) {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        printf("Motor 1 Forward - Encoder Value: %ld\n", encoderValue_1);

        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
        printf("Motor 2 Forward - Encoder Value: %ld\n", encoderValue_2);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

// Task para ir para tras
void motortras(void *arg) {
    while (1) {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        printf("Motor 1 Reverse - Encoder Value: %ld\n", encoderValue_1);

        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
        printf("Motor 2 Reverse - Encoder Value: %ld\n", encoderValue_2);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void motorparar(void*arg) {
    while (1) {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 0);
        printf("Motor 1 Reverse - Encoder Value: %ld\n", encoderValue_1);

        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 0);
        printf("Motor 2 Reverse - Encoder Value: %ld\n", encoderValue_2);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void motordireita(void*arg) {
    while (1) {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        printf("Motor 1 Reverse - Encoder Value: %ld\n", encoderValue_1);

        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
        printf("Motor 2 Reverse - Encoder Value: %ld\n", encoderValue_2);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void motoresquerda(void*arg) {
    while (1) {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        printf("Motor 1 Reverse - Encoder Value: %ld\n", encoderValue_1);

        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
        printf("Motor 2 Reverse - Encoder Value: %ld\n", encoderValue_2);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void app_main(void) {
    // Configuração dos pinos do Motor 1
    esp_rom_gpio_pad_select_gpio(MotFwd_1);
    esp_rom_gpio_pad_select_gpio(MotRev_1);
    gpio_set_direction(MotFwd_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MotRev_1, GPIO_MODE_OUTPUT);

    // Configuração dos pinos do Motor 2
    esp_rom_gpio_pad_select_gpio(MotFwd_2);
    esp_rom_gpio_pad_select_gpio(MotRev_2);
    gpio_set_direction(MotFwd_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MotRev_2, GPIO_MODE_OUTPUT);

    // Configuração dos encoders
    esp_rom_gpio_pad_select_gpio(encoderPin1_1);
    esp_rom_gpio_pad_select_gpio(encoderPin2_1);
    gpio_set_direction(encoderPin1_1, GPIO_MODE_INPUT);
    gpio_set_direction(encoderPin2_1, GPIO_MODE_INPUT);

    esp_rom_gpio_pad_select_gpio(encoderPin1_2);
    esp_rom_gpio_pad_select_gpio(encoderPin2_2);
    gpio_set_direction(encoderPin1_2, GPIO_MODE_INPUT);
    gpio_set_direction(encoderPin2_2, GPIO_MODE_INPUT);

    //instalar a interrupt no caso
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    // Configuração das interrupções dos encoders
    gpio_set_intr_type(encoderPin1_1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(encoderPin2_1, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(encoderPin1_2, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(encoderPin2_2, GPIO_INTR_ANYEDGE);

    gpio_isr_handler_add(encoderPin1_1, updateEncoder_1, NULL);
    gpio_isr_handler_add(encoderPin2_1, updateEncoder_1, NULL);
    gpio_isr_handler_add(encoderPin1_2, updateEncoder_2, NULL);
    gpio_isr_handler_add(encoderPin2_2, updateEncoder_2, NULL);

    // Criação das tasks para controlar os motores
    xTaskCreate(motorfrent, "motorfrent", 4096, NULL, 10, &motorfrenttask);
    xTaskCreate(motortras, "motortras", 4096, NULL, 10, &motortrastask);
    xTaskCreate(motorparar, "pararmotor", 4096, NULL, 10, &motorparartask);
    xTaskCreate(motordireita, "motordireita", 4096, NULL, 10, &motordireitatask);
    xTaskCreate(motoresquerda, "motoresquerda", 4096, NULL, 10, &motoresquerdatask);

    //suspensao das task para rodar so uma 
    vTaskSuspend(motortrastask);
    vTaskSuspend(motorparartask);
    vTaskSuspend(motordireitatask);
    vTaskSuspend(motoresquerdatask);

    // Criar um timer que dispara a cada 5 sec (5000 milissegundos)
    xTimer = xTimerCreate("TimerTroca", pdMS_TO_TICKS(5000), pdTRUE, (void *)0, vTimerCallback);

    // Iniciar o timer
    if (xTimer != NULL) {
        xTimerStart(xTimer, 0);
    }
}
