#include "pinout.h"
#include "motor.c"
#include "tofSetup.c"
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
    while(1){
        forward();
        vTaskDelay(1000/portTICK_PERIOD_MS);
        back();
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    
}

void app_main(void) {
    xTaskCreate(&ToFRead,"ToF",1024,NULL,1,NULL);
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}