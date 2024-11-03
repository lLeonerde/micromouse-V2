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
        //distance1 = vl53l0x_readRangeContinuousMillimeters(sensor1);
        distance2 = vl53l0x_readRangeContinuousMillimeters(sensor2);
        distance3 = vl53l0x_readRangeContinuousMillimeters(sensor3);
        distance4 = vl53l0x_readRangeContinuousMillimeters(sensor4);
        distance5 = vl53l0x_readRangeContinuousMillimeters(sensor5);

        //printf("Distância Sensor 1: %d mm\n", distance1);
        //printf("Distância Sensor 2: %d mm\n", distance2);
        //printf("Distância Sensor 3: %d mm\n", distance3);
        //printf("Distância Sensor 4: %d mm\n", distance4);
        //printf("Distância Sensor 5: %d mm\n", distance5);

        vTaskDelay(1 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}



//PASSO DO MOTOR = 820-824
void motorTask(){
    encoderPrint();
    
    while (1){
        control_motor_with_pid();
        encoderPrint();
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
    
}




void app_main(void) {

    
    setup();
    vTaskDelay(10/portTICK_PERIOD_MS);
    setupToF();
    //vTaskDelay(1000/portTICK_PERIOD_MS);
    xTaskCreate(&ToFRead,"ToF",2048,NULL,1,NULL);
    xTaskCreate(&motorTask,"motor",2048,NULL,3,NULL);
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}