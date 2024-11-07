#include "pinout.h"
//#include "motor.c"
#include "tofSetup.c"
#include "algorithm.c"

uint16_t frente = 0;
uint16_t esquerda = 0;
uint16_t direita = 0;
uint16_t diagonalE = 0;
uint16_t diagonalD = 0;
void ToFRead(){
    
    while(1){
        direita = vl53l0x_readRangeContinuousMillimeters(sensor1);
        diagonalD = vl53l0x_readRangeContinuousMillimeters(sensor2);
        frente = vl53l0x_readRangeContinuousMillimeters(sensor3);
        esquerda = vl53l0x_readRangeContinuousMillimeters(sensor4);
        diagonalE = vl53l0x_readRangeContinuousMillimeters(sensor5);
        vTaskDelay(20 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}



//PASSO DO MOTOR = 820-824

void explore_maze(){
    MazeState state;
    initMazeState(&state);  // Inicializa o estado do labirinto
    while (!state.targetReached) {
        ESP_LOGI(__func__,"test");
        detectWalls(&state,esquerda,direita,frente);  // Detecta paredes ao redor do robô 
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
    //vTaskDelay(1000/portTICK_PERIOD_MS);
    xTaskCreate(&ToFRead,"ToF",2048,NULL,1,NULL);
    vTaskDelay(500 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    xTaskCreate(&explore_maze,"Maze",6128,NULL,1,NULL);
    

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay para atualizar leituras
    }
}