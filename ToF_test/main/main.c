#include "vl53l0x.h"
#include "driver/i2c.h"
#include "esp_log.h"


#define I2C_MASTER_SCL_IO    21    // Defina o pino SCL
#define I2C_MASTER_SDA_IO    22    // Defina o pino SDA
#define I2C_HOST  0


void app_main(void) {
    
    // Configurar e inicializar o Sensor 1
    int8_t XSHUT_PIN_SENSOR_1 = 15; // Se não tiver controle de GPIO para xshut, use -1
    uint8_t address = 0x29; // Endereço padrão do VL53L0X
    uint8_t io_2v8 = 1; // Use 1 para I/O de 2.8V
    /*
    vl53l0x_t *sensor1 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_1, address, io_2v8);
    if (sensor1 == NULL) {
        printf("Falha na configuração do VL53L0X\n");
        return;
    }

    gpio_set_level(XSHUT_PIN_SENSOR_1, 0);  // desliga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    vl53l0x_init(sensor1);

    vl53l0x_setAddress(sensor1, 0x30); 
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    gpio_set_level(XSHUT_PIN_SENSOR_1, 1);  // Liga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    printf("Sensor no pino %d ativado.\n", XSHUT_PIN_SENSOR_1);
    
    // Configurar e inicializar Sensor 2
    int8_t XSHUT_PIN_SENSOR_2 = 4; // Se não tiver controle de GPIO para xshut, use -1
    vl53l0x_t *sensor2 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_2, address, io_2v8);
    if (sensor2 == NULL) {
        printf("Falha na configuração do VL53L0X\n");
        
    }

    gpio_set_level(XSHUT_PIN_SENSOR_2, 0);  // desliga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    vl53l0x_init(sensor2);

    vl53l0x_setAddress(sensor2, 0x31); 
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    gpio_set_level(XSHUT_PIN_SENSOR_2, 1);  // Liga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    printf("Sensor no pino %d ativado.\n", XSHUT_PIN_SENSOR_2);

// Configurar e inicializar Sensor 3
    int8_t XSHUT_PIN_SENSOR_3 = 23; // Se não tiver controle de GPIO para xshut, use -1
    vl53l0x_t *sensor3 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_3, address, io_2v8);
    if (sensor3 == NULL) {
        printf("Falha na configuração do VL53L0X\n");
        
    }

    gpio_set_level(XSHUT_PIN_SENSOR_3, 0);  // desliga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    vl53l0x_init(sensor3);

    vl53l0x_setAddress(sensor3, 0x32); 
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    gpio_set_level(XSHUT_PIN_SENSOR_3, 1);  // Liga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    printf("Sensor no pino %d ativado.\n", XSHUT_PIN_SENSOR_3);
*/
// Configurar e inicializar Sensor 4
    int8_t XSHUT_PIN_SENSOR_4 = 23; // Se não tiver controle de GPIO para xshut, use -1
    vl53l0x_t *sensor4 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_4, address, io_2v8);
    if (sensor4 == NULL) {
        printf("Falha na configuração do VL53L0X\n");
        
    }

    gpio_set_level(XSHUT_PIN_SENSOR_4, 0);  // desliga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    vl53l0x_init(sensor4);

    vl53l0x_setAddress(sensor4, 0x33); 
    uint8_t addr4 = vl53l0x_getAddress(sensor4);
    ESP_LOGI(__func__,"addr4 = %d",addr4);
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    gpio_set_level(XSHUT_PIN_SENSOR_4, 1);  // Liga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    printf("Sensor no pino %d ativado.\n", XSHUT_PIN_SENSOR_4);
    
/*
// Configurar e inicializar Sensor 5
    int8_t XSHUT_PIN_SENSOR_5 = 19; // Se não tiver controle de GPIO para xshut, use -1
    vl53l0x_t *sensor5 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_5, address, io_2v8);
    if (sensor5 == NULL) {
        printf("Falha na configuração do VL53L0X\n");
        
    }

    gpio_set_level(XSHUT_PIN_SENSOR_5, 0);  // desliga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    vl53l0x_init(sensor5);

    vl53l0x_setAddress(sensor5, 0x34); 
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    gpio_set_level(XSHUT_PIN_SENSOR_5, 1);  // Liga o sensor
    vTaskDelay(10 / portTICK_PERIOD_MS); // Aguardar a ativação do sensor
    printf("Sensor no pino %d ativado.\n", XSHUT_PIN_SENSOR_5);
*/
    //vl53l0x_startContinuous(sensor1, 0);
    //vl53l0x_startContinuous(sensor2, 0);
    //vl53l0x_startContinuous(sensor3, 0);
    vl53l0x_startContinuous(sensor4, 0);
    //vl53l0x_startContinuous(sensor5, 0);
    
    while (1) {
        //uint16_t distance1 = vl53l0x_readRangeContinuousMillimeters(sensor1);
        //uint16_t distance2 = vl53l0x_readRangeContinuousMillimeters(sensor2);
        //uint16_t distance3 = vl53l0x_readRangeContinuousMillimeters(sensor3);
        uint16_t distance4 = vl53l0x_readRangeContinuousMillimeters(sensor4);
        //uint16_t distance5 = vl53l0x_readRangeContinuousMillimeters(sensor5);

        //printf("Distância Sensor 1: %d mm\n", distance1);
        //printf("Distância Sensor 2: %d mm\n", distance2);
        //printf("Distância Sensor 3: %d mm\n", distance3);
        printf("Distância Sensor 3: %d mm\n", distance4);
        //printf("Distância Sensor 3: %d mm\n", distance5);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay de 100ms
    }

    //vl53l0x_stopContinuous(sensor1);
    //vl53l0x_end(sensor1);

    //vl53l0x_stopContinuous(sensor2);
    //vl53l0x_end(sensor2);

    //vl53l0x_stopContinuous(sensor3);
    //vl53l0x_end(sensor3);

    vl53l0x_stopContinuous(sensor4);
    vl53l0x_end(sensor4);

    //vl53l0x_stopContinuous(sensor5);
    //vl53l0x_end(sensor5);
}