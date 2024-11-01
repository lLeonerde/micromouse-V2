#include "pinout.h"
uint16_t lastEncoded_1 = 0;
uint32_t encoderValue_1 = 0;
uint16_t lastEncoded_2 = 0;
uint32_t encoderValue_2 = 0;

// Função de interrupção para o encoder 1
void IRAM_ATTR updateEncoder_1() {
    uint8_t MSB = gpio_get_level(encoderPin1_1);
    uint8_t LSB = gpio_get_level(encoderPin2_1);

    uint8_t encoded = (MSB << 1) | LSB;
    uint16_t sum = (lastEncoded_1 << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_1--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_1++;

    lastEncoded_1 = encoded;
}

void IRAM_ATTR updateEncoder_2() {
    uint8_t MSB = gpio_get_level(encoderPin1_2);
    uint8_t LSB = gpio_get_level(encoderPin2_2);

    uint8_t encoded = (MSB << 1) | LSB;
    uint16_t sum = (lastEncoded_2 << 2) | encoded;

    if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_2--;
    if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_2++;

    lastEncoded_2 = encoded;
}

void encoderPrint(){
    ESP_LOGI(__func__,"Encoder1: %ld",encoderValue_1);
    ESP_LOGI(__func__,"Encoder2: %ld",encoderValue_2);
}

// Task para ir para frente
void forward() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);

        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);

}

// Task para ir para tras
void back() {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
}

void stop() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 1);
}

void right() {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
}

void left() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
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

