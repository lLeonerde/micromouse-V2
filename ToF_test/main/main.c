#include "vl53l0x.h"
#include "driver/i2c.h"
#include "esp_log.h"


#define I2C_MASTER_SCL_IO    18    // Defina o pino SCL
#define I2C_MASTER_SDA_IO    19    // Defina o pino SDA
#define I2C_HOST  0


void app_main(void) {
    
    // Configurar e inicializar o VL53L0X
    int8_t xshut = -1; // Se não tiver controle de GPIO para xshut, use -1
    uint8_t address = 0x29; // Endereço padrão do VL53L0X
    uint8_t io_2v8 = 1; // Use 1 para I/O de 2.8V
    ESP_LOGI(__func__,"test");
    vl53l0x_t *sensor = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, xshut, address, io_2v8);
    ESP_LOGI(__func__,"test2");
    if (sensor == NULL) {
        printf("Falha na configuração do VL53L0X\n");
        return;
    }
    const char *error = vl53l0x_init(sensor);
    if (error != NULL) {
        printf("Erro de inicialização: %s\n", error);
        vl53l0x_end(sensor);
        return;
    }

    vl53l0x_startContinuous(sensor, 0);

    while(1){
        uint16_t distance = vl53l0x_readRangeContinuousMillimeters(sensor);
        printf("Distância: %d mm\n", distance);
        vTaskDelay(1 / portTICK_PERIOD_MS); // Delay de 100ms
    }

    vl53l0x_stopContinuous(sensor);
    vl53l0x_end(sensor);
}