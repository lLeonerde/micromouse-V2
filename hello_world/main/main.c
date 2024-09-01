#include "vl53l0x.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO    18    // Defina o pino SCL
#define I2C_MASTER_SDA_IO    19    // Defina o pino SDA
#define I2C_MASTER_NUM       I2C_NUM_0   // I2C port
#define I2C_MASTER_FREQ_HZ   100000      // Frequência do I2C

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void) {
    i2c_master_init();

    // Configurar e inicializar o VL53L0X
    int8_t xshut = -1; // Se não tiver controle de GPIO para xshut, use -1
    uint8_t address = 0x29; // Endereço padrão do VL53L0X
    uint8_t io_2v8 = 1; // Use 1 para I/O de 2.8V

    vl53l0x_t *sensor = vl53l0x_config(I2C_MASTER_NUM, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, xshut, address, io_2v8);
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

    for (int i = 0; i < 10; i++) {
        uint16_t distance = vl53l0x_readRangeContinuousMillimeters(sensor);
        printf("Distância: %d mm\n", distance);
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay de 100ms
    }

    vl53l0x_stopContinuous(sensor);
    vl53l0x_end(sensor);
}