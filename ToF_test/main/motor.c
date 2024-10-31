#include "pinout.h"
volatile int lastEncoded_1 = 0;
volatile long encoderValue_1 = 0;
volatile int lastEncoded_2 = 0;
volatile long encoderValue_2 = 0;

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

void IRAM_ATTR updateEncoder_2() {
    int MSB = gpio_get_level(encoderPin1_2);
    int LSB = gpio_get_level(encoderPin2_2);

    int encoded = (MSB << 1) | LSB;
    int sum = (lastEncoded_2 << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_1--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_1++;

    lastEncoded_2 = encoded;
}

// Task para ir para frente
void forward() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        printf("Motor 1 Forward - Encoder Value: %ld\n", encoderValue_1);

        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
        printf("Motor 2 Forward - Encoder Value: %ld\n", encoderValue_2);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop

}

// Task para ir para tras
void back() {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
}

void stop() {
    while (1) {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 0);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void right() {
    while (1) {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void left() {
    while (1) {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Aguardar um pouco para não travar o loop
    }
}

void setup(){
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

}

