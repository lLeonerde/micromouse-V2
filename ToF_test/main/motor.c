#include "pinout.h"
uint16_t lastEncoded_1 = 0;
uint32_t encoderValue_1 = 0;
uint16_t lastEncoded_2 = 0;
uint32_t encoderValue_2 = 0;





void encoderPrint(){
    ESP_LOGI(__func__,"Encoder1: %ld",encoderValue_1);
    ESP_LOGI(__func__,"Encoder2: %ld",encoderValue_2);
}

// Task para ir para frente
void IRAM_ATTR forward() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
}

// Task para ir para tras
void IRAM_ATTR back() {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
}

void IRAM_ATTR freeWheel(){
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 0);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 0);
}

void IRAM_ATTR stop() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 1);
}

void IRAM_ATTR right() {
        gpio_set_level(MotFwd_1, 1);
        gpio_set_level(MotRev_1, 0);
        gpio_set_level(MotFwd_2, 0);
        gpio_set_level(MotRev_2, 1);
}

void IRAM_ATTR left() {
        gpio_set_level(MotFwd_1, 0);
        gpio_set_level(MotRev_1, 1);
        gpio_set_level(MotFwd_2, 1);
        gpio_set_level(MotRev_2, 0);
}

void step(){
        forward();
        vTaskDelay(200/portTICK_PERIOD_MS);
        stop();
}

// Função de interrupção para o encoder 1
void IRAM_ATTR updateEncoder_1() {
        uint8_t MSB = gpio_get_level(encoderPin1_1);
        uint8_t LSB = gpio_get_level(encoderPin2_1);

        uint8_t encoded = (MSB << 1) | LSB;
        uint16_t sum = (lastEncoded_1 << 2) | encoded;
                
        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_1--;
        if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_1++;

        //if(encoderValue_1 > 800-MOTOR_OFSET){
        //        stop();
        //}

        lastEncoded_1 = encoded;
}

void IRAM_ATTR updateEncoder_2() {
        uint8_t MSB = gpio_get_level(encoderPin1_2);
        uint8_t LSB = gpio_get_level(encoderPin2_2);

        uint8_t encoded = (MSB << 1) | LSB;
        uint16_t sum = (lastEncoded_2 << 2) | encoded;

        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_2++;
        if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_2--;

        //if(encoderValue_1 > 800-MOTOR_OFSET){
        //        stop();
        //}

        lastEncoded_2 = encoded;
}


void setup(){
        gpio_config_t out_conf = {
        .intr_type = GPIO_INTR_DISABLE,        // Desativa interrupções
        .mode = GPIO_MODE_OUTPUT,              // Configura o pino como saída
        .pin_bit_mask = (1ULL << MotFwd_1) |
                        (1ULL << MotRev_1) |
                        (1ULL << MotFwd_2) |
                        (1ULL << MotRev_2), // Configura o pino específico
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Desativa pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE      // Desativa pull-up
        };
        gpio_config_t in_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,        // Desativa interrupções
        .mode = GPIO_MODE_INPUT,              // Configura o pino como saída
        .pin_bit_mask = (1ULL << encoderPin1_1) |
                        (1ULL << encoderPin2_1) |
                        (1ULL << encoderPin1_2) |
                        (1ULL << encoderPin2_2), // Configura o pino específico
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Desativa pull-down
        .pull_up_en = GPIO_PULLUP_DISABLE      // Desativa pull-up
        };
        gpio_config(&out_conf);
        gpio_config(&in_conf);
        esp_rom_gpio_pad_select_gpio(MotFwd_1);
        esp_rom_gpio_pad_select_gpio(MotRev_1);
        esp_rom_gpio_pad_select_gpio(MotFwd_2);
        esp_rom_gpio_pad_select_gpio(MotRev_2);
        esp_rom_gpio_pad_select_gpio(encoderPin1_1);
        esp_rom_gpio_pad_select_gpio(encoderPin2_1);
        esp_rom_gpio_pad_select_gpio(encoderPin1_2);
        esp_rom_gpio_pad_select_gpio(encoderPin2_2);

        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

        gpio_isr_handler_add(encoderPin1_1, updateEncoder_1, NULL);
        gpio_isr_handler_add(encoderPin2_1, updateEncoder_1, NULL);
        gpio_isr_handler_add(encoderPin1_2, updateEncoder_2, NULL);
        gpio_isr_handler_add(encoderPin2_2, updateEncoder_2, NULL);
}

