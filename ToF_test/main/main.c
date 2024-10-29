#include "vl53l0x.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    21    // Defina o pino SCL
#define I2C_MASTER_SDA_IO    22    // Defina o pino SDA
#define I2C_HOST  0

// Defina os pinos XSHUT para cada sensor
//#define XSHUT_PIN_SENSOR_1   15
#define XSHUT_PIN_SENSOR_2   4
#define XSHUT_PIN_SENSOR_3   23
//#define XSHUT_PIN_SENSOR_4   18
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

void app_main(void) {
    esp_rom_gpio_pad_select_gpio(XSHUT_PIN_SENSOR_2);
    gpio_set_direction(XSHUT_PIN_SENSOR_2, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(XSHUT_PIN_SENSOR_3);
    gpio_set_direction(XSHUT_PIN_SENSOR_3, GPIO_MODE_OUTPUT);
    // Desativar todos os sensores no início
    //deactivateSensor(XSHUT_PIN_SENSOR_1);
    deactivateSensor(XSHUT_PIN_SENSOR_2);
    deactivateSensor(XSHUT_PIN_SENSOR_3);
   // deactivateSensor(XSHUT_PIN_SENSOR_4);
    //deactivateSensor(XSHUT_PIN_SENSOR_5);

    uint8_t io_2v8 = 1; // Use 1 para I/O de 2.8V

    // Ativar e configurar o sensor 1
    //activateSensor(XSHUT_PIN_SENSOR_1);
    //uint8_t address_1 = 0x30;  // Novo endereço para o sensor 1
    //vl53l0x_t *sensor1 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, 15, 0x29, io_2v8);
    //if (sensor1 == NULL) {
    //    printf("Falha na configuração do Sensor 1\n");
   //     return;
    //}
    
   // setSensorAddress(sensor1, XSHUT_PIN_SENSOR_1, address_1);  // passar o XSHUT_PIN
    //vTaskDelay(100 / portTICK_PERIOD_MS);  // Aguardar 100ms para garantir a mudança
    //deactivateSensor(XSHUT_PIN_SENSOR_1);  // Desativa o sensor 1 para evitar conflito de endereços
    
    // Ativar e configurar o sensor 2
    activateSensor(XSHUT_PIN_SENSOR_2);

    uint8_t address_2 = 0x31;  // Novo endereço para o sensor 2
    vl53l0x_t *sensor2 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_2, 0x29, io_2v8);

    if (sensor2 == NULL) {
        printf("Fail in sensor 2\n");
        return;
    }
    setSensorAddress(sensor2, XSHUT_PIN_SENSOR_2, address_2);  // passar o XSHUT_PIN
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Aguardar 100ms para garantir a mudança
    deactivateSensor(XSHUT_PIN_SENSOR_2);  // Desativa o sensor 2 para evitar conflito de endereços
    // Ativar e configurar o sensor 3
    activateSensor(XSHUT_PIN_SENSOR_3);
    uint8_t address_3 = 0x33;  // Novo endereço para o sensor 3
    
    vl53l0x_t *sensor3 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_3, 0x29, io_2v8);
    if (sensor3 == NULL) {
        printf("Fail in sensor 3\n");
        return;
    }
    setSensorAddress(sensor3, XSHUT_PIN_SENSOR_3, address_3);  // passar o XSHUT_PIN
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Aguardar 100ms para garantir a mudança
    deactivateSensor(XSHUT_PIN_SENSOR_3);  // Desativa o sensor 3 para evitar conflito de endereços

    
    /*
        // Ativar e configurar o sensor 4
    activateSensor(XSHUT_PIN_SENSOR_4);
    uint8_t address_4 = 0x34;  // Novo endereço para o sensor 4
    vl53l0x_t *sensor4 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_4, 0x29, io_2v8);
    if (sensor4 == NULL) {
        printf("Falha na configuração do Sensor 4\n");
        return;
    }
    setSensorAddress(sensor4, XSHUT_PIN_SENSOR_4, address_4);  // passar o XSHUT_PIN
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Aguardar 100ms para garantir a mudança
    deactivateSensor(XSHUT_PIN_SENSOR_4);  // Desativa o sensor 4 para evitar conflito de endereços



    
        // Ativar e configurar o sensor 5
    activateSensor(XSHUT_PIN_SENSOR_5);
    uint8_t address_5 = 0x35;  // Novo endereço para o sensor 5
    vl53l0x_t *sensor5 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_5, 0x29, io_2v8);
    if (sensor5 == NULL) {
        printf("Falha na configuração do Sensor 5\n");
        return;
    }
    setSensorAddress(sensor5, XSHUT_PIN_SENSOR_5, address_5);  // passar o XSHUT_PIN
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Aguardar 100ms para garantir a mudança
    deactivateSensor(XSHUT_PIN_SENSOR_5);  // Desativa o sensor 4 para evitar conflito de endereços

    */

    // Iniciar leituras dos sensores
    //activateSensor(XSHUT_PIN_SENSOR_1);
    activateSensor(XSHUT_PIN_SENSOR_2);
    activateSensor(XSHUT_PIN_SENSOR_3);
    //activateSensor(XSHUT_PIN_SENSOR_4);
    //activateSensor(XSHUT_PIN_SENSOR_5);
    
    int test2 = vl53l0x_getAddress(sensor2);
    int test3 = vl53l0x_getAddress(sensor3);

    
    //vl53l0x_startContinuous(sensor1, 0);
    //vl53l0x_startContinuous(sensor2, 0);
    //vl53l0x_startContinuous(sensor3, 0);
    //vl53l0x_startContinuous(sensor4, 0);
    //vl53l0x_startContinuous(sensor5, 0);
    
    while (1) {
        //uint16_t distance1 = vl53l0x_readRangeContinuousMillimeters(sensor1);
        //uint16_t distance2 = vl53l0x_readRangeContinuousMillimeters(sensor2);
        //uint16_t distance3 = vl53l0x_readRangeContinuousMillimeters(sensor3);
        //uint16_t distance4 = vl53l0x_readRangeContinuousMillimeters(sensor4);
        //uint16_t distance5 = vl53l0x_readRangeContinuousMillimeters(sensor5);

        //printf("Distância Sensor 1: %d mm\n", distance1);
        //printf("Distância Sensor 2: %d mm\n", distance2);
        //printf("Distância Sensor 3: %d mm\n", distance3);
        //printf("Distância Sensor 3: %d mm\n", distance4);
        //printf("Distância Sensor 3: %d mm\n", distance5);

        //vTaskDelay(1 / portTICK_PERIOD_MS); // Delay de 100ms
    }
}
