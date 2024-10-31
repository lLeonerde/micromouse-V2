#include "pinout.h"
#include "motor.c"
vl53l0x_t *sensor1, *sensor2, *sensor3,*sensor4,*sensor5;
void setupToF(){
    i2c_master_init();

    configure_gpio(XSHUT_PIN_SENSOR_1);
    configure_gpio(XSHUT_PIN_SENSOR_2);
    configure_gpio(XSHUT_PIN_SENSOR_3);
    configure_gpio(XSHUT_PIN_SENSOR_4);

    uint8_t io_2v8 = 1; // 1 for I/O 2.8V
    uint8_t initial_address = 0x29; // Default address

    // Config/create each sensor
    sensor1 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_1, initial_address, io_2v8);
    sensor2 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_2, initial_address, io_2v8);
    sensor3 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_3, initial_address, io_2v8);
    sensor4 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_4, initial_address, io_2v8);
    sensor5 = vl53l0x_config(I2C_HOST, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, XSHUT_PIN_SENSOR_5, initial_address, io_2v8);

    if (sensor1 == NULL || sensor2 == NULL || sensor3 == NULL || sensor4 == NULL|| sensor5 == NULL) {
        ESP_LOGE(__func__,"Error configuring 1 or more sensors\n");
        return;
    }
    //starting them
    setupSensor(sensor1, XSHUT_PIN_SENSOR_1, 0x30);
    setupSensor(sensor2, XSHUT_PIN_SENSOR_2, 0x31);
    setupSensor(sensor3, XSHUT_PIN_SENSOR_3, 0x32);
    setupSensor(sensor4, XSHUT_PIN_SENSOR_4, 0x33);
    setupSensor(sensor5, XSHUT_PIN_SENSOR_5, 0x33);

    vl53l0x_startContinuous(sensor1, 2);
    vl53l0x_startContinuous(sensor2, 2);
    vl53l0x_startContinuous(sensor3, 2);
    //vl53l0x_startContinuous(sensor4, 2);
}

void ToFRead(){
    uint16_t distance1 = 0;
    uint16_t distance2 = 0;
    uint16_t distance3 = 0;
    uint16_t distance4 = 0;
    uint16_t distance5 = 0;
    while(1){
        distance1 = vl53l0x_readRangeContinuousMillimeters(sensor1);
        distance2 = vl53l0x_readRangeContinuousMillimeters(sensor2);
        distance3 = vl53l0x_readRangeContinuousMillimeters(sensor3);
        distance4 = vl53l0x_readRangeContinuousMillimeters(sensor4);
        distance5 = vl53l0x_readRangeContinuousMillimeters(sensor5);

        printf("Distância Sensor 1: %d mm\n", distance1);
        printf("Distância Sensor 2: %d mm\n", distance2);
        printf("Distância Sensor 3: %d mm\n", distance3);
        //printf("Distância Sensor 4: %d mm\n", distance4);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}

void motorTask(){
    forward();
}

void app_main(void) {
    xTaskCreate(&ToFRead,"ToF",1024,NULL,1,NULL);
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}