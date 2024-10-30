#include "vl53l0x.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    21    // Defina o pino SCL
#define I2C_MASTER_SDA_IO    22    // Defina o pino SDA
#define I2C_HOST      I2C_NUM_0   // I2C port
#define I2C_MASTER_FREQ_HZ   300000      // Frequência do I2C

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_HOST, &conf);
    i2c_driver_install(I2C_HOST, conf.mode, 0, 0, 0);
    i2c_set_timeout (I2C_HOST, 80000);       // Clock stretching
    i2c_filter_enable (I2C_HOST, 5);
}

// Defina os pinos XSHUT para cada sensor
#define XSHUT_PIN_SENSOR_1   19 //15
#define XSHUT_PIN_SENSOR_2   4
#define XSHUT_PIN_SENSOR_3   23
#define XSHUT_PIN_SENSOR_4   18
//#define XSHUT_PIN_SENSOR_5   19


// Declaração das funções
void activateSensor(uint8_t xshut_pin);
void deactivateSensor(uint8_t xshut_pin);

// Função para alterar o endereço I2C do sensor
void setSensorAddress(vl53l0x_t *sensor, uint8_t xshut_pin, uint8_t new_address) {
    deactivateSensor(xshut_pin);    // Desativa o sensor para começar a configuração
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Aguardar a desativação
    
    activateSensor(xshut_pin);  // Ativa o sensor
    vl53l0x_init(sensor);       // Inicializa o sensor (caso necessário)
    vl53l0x_setAddress(sensor, new_address); // Configura o novo endereço
    printf("sensor pin %d configured for 0x%02X.\n", xshut_pin, new_address);
}

// Função para ativar o sensor
void activateSensor(uint8_t xshut_pin) {
    gpio_set_level(xshut_pin, 1);  // Liga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    printf("sensor pin %d activated.\n", xshut_pin);
}

// Função para desativar o sensor
void deactivateSensor(uint8_t xshut_pin) {
    gpio_set_level(xshut_pin, 0);  // Desliga o sensor
    printf("sensor pin %d deactivated.\n", xshut_pin);
}


void configure_gpio(uint8_t xshut_pin) {
    gpio_reset_pin(xshut_pin); // Reseta o pino para evitar configurações antigas
    gpio_set_direction(xshut_pin, GPIO_MODE_OUTPUT); // Define o pino como saída
    deactivateSensor(xshut_pin); // Inicialmente desativa o sensor
}
// Função para configurar o endereço e iniciar cada sensor
void setupSensor(vl53l0x_t *sensor, uint8_t xshut_pin, uint8_t new_address) {
    deactivateSensor(xshut_pin);    
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Aguardar a desativação
    
    activateSensor(xshut_pin);   
    if (vl53l0x_init(sensor) == ESP_OK) {
        vl53l0x_setAddress(sensor, new_address); // Configura o novo endereço
        printf("Sensor pino %d configurado para 0x%02X.\n", xshut_pin, new_address);
    } else {
        printf("Erro ao inicializar o sensor no pino %d\n", xshut_pin);
    }
}



void app_main(void) {
    i2c_master_init();

    // Configura os pinos XSHUT de cada sensor
    configure_gpio(XSHUT_PIN_SENSOR_1);
    configure_gpio(XSHUT_PIN_SENSOR_2);
    configure_gpio(XSHUT_PIN_SENSOR_3);
    configure_gpio(XSHUT_PIN_SENSOR_4);

    // Configura cada sensor VL53L0X
    uint8_t io_2v8 = 1; // Use 1 para I/O de 2.8V
    uint8_t initial_address = 0x29; // Endereço I2C padrão de fábrica

    // Cria e configura cada sensor
    vl53l0x_t *sensor1 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_1, initial_address, io_2v8);
    vl53l0x_t *sensor2 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_2, initial_address, io_2v8);
    vl53l0x_t *sensor3 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_3, initial_address, io_2v8);
    vl53l0x_t *sensor4 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_4, initial_address, io_2v8);

    if (sensor1 == NULL || sensor2 == NULL || sensor3 == NULL || sensor4 == NULL) {
        printf("Erro na configuração de um ou mais sensores VL53L0X\n");
        return;
    }

    // Configura endereços I2C individuais para cada sensor
    setupSensor(sensor1, XSHUT_PIN_SENSOR_1, 0x30);
    setupSensor(sensor2, XSHUT_PIN_SENSOR_2, 0x31);
    setupSensor(sensor3, XSHUT_PIN_SENSOR_3, 0x32);
    //setupSensor(sensor4, XSHUT_PIN_SENSOR_4, 0x29);
    // Ativar a leitura contínua de distância de cada sensor
    vl53l0x_startContinuous(sensor1, 2);
    vl53l0x_startContinuous(sensor2, 2);
    vl53l0x_startContinuous(sensor3, 2);
    //vl53l0x_startContinuous(sensor4, 2);
    
    while (1) {
        // Leitura de cada sensor
        uint16_t distance1 = vl53l0x_readRangeContinuousMillimeters(sensor1);
        uint16_t distance2 = vl53l0x_readRangeContinuousMillimeters(sensor2);
        uint16_t distance3 = vl53l0x_readRangeContinuousMillimeters(sensor3);
        //uint16_t distance4 = vl53l0x_readRangeContinuousMillimeters(sensor4);

        printf("Distância Sensor 1: %d mm\n", distance1);
        printf("Distância Sensor 2: %d mm\n", distance2);
        printf("Distância Sensor 3: %d mm\n", distance3);
        //printf("Distância Sensor 4: %d mm\n", distance4);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}