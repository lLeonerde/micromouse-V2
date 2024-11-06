#include "pinout.h"
int16_t lastEncoded_1 = 0;
long encoderValue_1 = 0;
int16_t lastEncoded_2 = 0;
long encoderValue_2 = 0;

// Variáveis do PID para Motor 1
double previous_error_1 = 0;
double integral_1 = 0;
bool motor_1_stopped = false; 

// Variáveis do PID para Motor 2
double previous_error_2 = 0;
double integral_2 = 0;
bool motor_2_stopped = false;  


void init_ledc() {
        // Configura o timer
        ledc_timer_config_t ledc_timer = {
                .speed_mode       = LEDC_MODE,
                .timer_num        = LEDC_TIMER,
                .duty_resolution  = LEDC_RESOLUTION,
                .freq_hz          = LEDC_FREQUENCY,
                .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&ledc_timer);

        // Configura os canais LEDC para cada direção do motor
        ledc_channel_config_t ledc_channel[] = {
                {
                .channel    = LEDC_CHANNEL_FWD_1,
                .duty       = 0,
                .gpio_num   = LEDC_OUTPUT_IO_FWD_1,
                .speed_mode = LEDC_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_TIMER
                },
                {
                .channel    = LEDC_CHANNEL_REV_1,
                .duty       = 0,
                .gpio_num   = LEDC_OUTPUT_IO_REV_1,
                .speed_mode = LEDC_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_TIMER
                },
                {
                .channel    = LEDC_CHANNEL_FWD_2,
                .duty       = 0,
                .gpio_num   = LEDC_OUTPUT_IO_FWD_2,
                .speed_mode = LEDC_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_TIMER
                },
                {
                .channel    = LEDC_CHANNEL_REV_2,
                .duty       = 0,
                .gpio_num   = LEDC_OUTPUT_IO_REV_2,
                .speed_mode = LEDC_MODE,
                .hpoint     = 0,
                .timer_sel  = LEDC_TIMER
                }
        };

        // Inicializa cada canal
        for (int ch = 0; ch < 4; ch++) {
                ledc_channel_config(&ledc_channel[ch]);
        }
}


void encoderPrint(){
        ESP_LOGI(__func__,"Encoder1: %ld",encoderValue_1);
        ESP_LOGI(__func__,"Encoder2: %ld",encoderValue_2);
}

void set_motor_speed(uint8_t motor, int8_t direction, uint32_t speed) {
    // Define o valor de duty cycle (entre 0 e 8191 para resolução de 13 bits)
        uint32_t duty = (speed * ((1 << LEDC_RESOLUTION) - 1)) / 100; // speed em %

        if (motor == 1) {
                if (direction == 0) { // Para frente
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FWD_1, duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FWD_1);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_REV_1, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_REV_1);
                } else if(direction == -1){ // freio
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_REV_1, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_REV_1);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FWD_1, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FWD_1);
                }else{
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_REV_1, duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_REV_1);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FWD_1, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FWD_1);
                }
                } else if (motor == 2) {
                if (direction == 0) { // Para frente
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FWD_2, duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FWD_2);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_REV_2, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_REV_2);
                } else if(direction == -1){ // Para trás
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_REV_2, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_REV_2);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FWD_2, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FWD_2);
                }else{
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_REV_2, duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_REV_2);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_FWD_2, 0);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_FWD_2);
                }
        }
}

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

        if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue_2++;
        if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue_2--;
        lastEncoded_2 = encoded;
}


int32_t calculate_pid(int target_value, int current_value, double *integral, double *previous_error, bool *motor_stopped) {
        double error = target_value - current_value;

        // Se o erro estiver dentro da tolerância, considera o alvo atingido
        if (abs(error) <= ERROR_TOLERANCE) {
                *motor_stopped = true;
                return 0;
        }

        *motor_stopped = false;

        // Termo Integral com limite para evitar "windup"
        *integral += error;
        // Termo Derivativo
        double derivative = error - *previous_error;
        *previous_error = error;

        // Cálculo do PID
        double output = KP * error + KI * (*integral) + KD * derivative;

        // Limita a saída na faixa de velocidade
        if (output > MAX_SPEED) output = MAX_SPEED;
        if (output < MIN_SPEED) output = MIN_SPEED;

        if (output > 0 && output < MIN_DUTY_CYCLE) output = 0;
        if (output < 0 && output > -MIN_DUTY_CYCLE) output = 0;

        ESP_LOGW("calculate_pid", "error: %f, integral: %f, derivative: %f, output: %f", error, *integral, derivative, output);

        return (int)output;
}

int16_t control_motor_with_pid(int32_t target1, int32_t target2) {
        int motor_speed_1, motor_speed_2;

        // Cálculo PID para cada motor
        motor_speed_1 = calculate_pid(target1, encoderValue_1, &integral_1, &previous_error_1, &motor_1_stopped);
        motor_speed_2 = calculate_pid(target2, encoderValue_2, &integral_2, &previous_error_2, &motor_2_stopped);

        // Controle do Motor 1
        if (motor_1_stopped) {
                //integral_1 = 0;
                set_motor_speed(2, -1, 0);  // Para o motor 1
                
        } else {
                if (motor_speed_1 >= 0) {
                set_motor_speed(2, 0, motor_speed_1);  // Motor 1 para frente
                } else {
                set_motor_speed(2, 1, -motor_speed_1); // Motor 1 para trás
                }
        }

        // Controle do Motor 2
        if (motor_2_stopped) {
                //integral_2 = 0;
                set_motor_speed(1, -1, 0);  // Para o motor 2
        } else {
                if (motor_speed_2 >= 0) {
                set_motor_speed(1, 0, motor_speed_2);  // Motor 2 para frente
                } else {
                set_motor_speed(1, 1, -motor_speed_2); // Motor 2 para trás
                }
        }

        if(motor_2_stopped == true && motor_1_stopped == true){
                return 0;
        }
        return 1;
}

void turnLeft(){
        uint8_t control = 1;
        ESP_LOGI(__func__,"left");
        encoderValue_1 = 0;
        encoderValue_2 = 0;
        integral_1 = 0;
        integral_2 = 0;
        previous_error_1 = 0;
        previous_error_2 = 0;
        while(control){
                control = control_motor_with_pid(-500, 500);
                vTaskDelay(10/portTICK_PERIOD_MS);
        }
                
}
void turnRight(){
        uint8_t control = 1;
        encoderValue_1 = 0;
        encoderValue_2 = 0;
        integral_1 = 0;
        integral_2 = 0;
        previous_error_1 = 0;
        previous_error_2 = 0;
        ESP_LOGI(__func__,"right");
        while(control){
                control = control_motor_with_pid(500, -500);
                vTaskDelay(10/portTICK_PERIOD_MS);
        }
}

void forward(){
        uint8_t control = 1;
        ESP_LOGI(__func__,"forward");
        encoderValue_1 = 0;
        encoderValue_2 = 0;
        integral_1 = 0;
        integral_2 = 0;
        previous_error_1 = 0;
        previous_error_2 = 0;
        while(control){
                control = control_motor_with_pid(1020, 1020);
                vTaskDelay(10/portTICK_PERIOD_MS);
        }
}

void setup(){
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
        gpio_config(&in_conf);
        esp_rom_gpio_pad_select_gpio(encoderPin1_1);
        esp_rom_gpio_pad_select_gpio(encoderPin2_1);
        esp_rom_gpio_pad_select_gpio(encoderPin1_2);
        esp_rom_gpio_pad_select_gpio(encoderPin2_2);

        gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

        gpio_isr_handler_add(encoderPin1_1, updateEncoder_1, NULL);
        gpio_isr_handler_add(encoderPin2_1, updateEncoder_1, NULL);
        gpio_isr_handler_add(encoderPin1_2, updateEncoder_2, NULL);
        gpio_isr_handler_add(encoderPin2_2, updateEncoder_2, NULL);
        init_ledc();
}

