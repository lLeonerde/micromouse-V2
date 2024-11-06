#include "pinout.h"
//#include "motor.c"
#include "tofSetup.c"
#include "algorithm.c"

uint16_t distance1 = 0;
uint16_t distance2 = 0;
uint16_t distance3 = 0;
uint16_t distance4 = 0;
uint16_t distance5 = 0;
void ToFRead(){
    
    while(1){
        distance1 = vl53l0x_readRangeContinuousMillimeters(sensor1);
        distance2 = vl53l0x_readRangeContinuousMillimeters(sensor2);
        distance3 = vl53l0x_readRangeContinuousMillimeters(sensor3);
        distance4 = vl53l0x_readRangeContinuousMillimeters(sensor4);
        distance5 = vl53l0x_readRangeContinuousMillimeters(sensor5);
        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}



//PASSO DO MOTOR = 820-824

void explore_maze(){
    MazeState state;
    initMazeState(&state);  // Inicializa o estado do labirinto
    while (!state.targetReached) {
        ESP_LOGI(__func__,"test");
        detectWalls(&state,distance4,distance3,distance1);  // Detecta paredes ao redor do robô 
        updateDistances(&state);  // Atualiza as distâncias no labirinto
        moveRobot(&state);  // Move o robô para a próxima célula
        // Verifica se o robô chegou ao centro do labirinto
        if (state.current.x == MAZE_SIZE/2 && state.current.y == MAZE_SIZE/2) {
            state.targetReached = true;  // Marca o alvo como alcançado
        }
       
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    printf("Caminho encontrado:\n");
    while(1){
            vTaskDelay(10/portTICK_PERIOD_MS);
    }
}



void app_main(void) {

    
    setup();
    vTaskDelay(10/portTICK_PERIOD_MS);
    setupToF();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    xTaskCreate(&ToFRead,"ToF",2048,NULL,1,NULL);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    xTaskCreate(&explore_maze,"Maze",6128,NULL,1,NULL);
    
    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}