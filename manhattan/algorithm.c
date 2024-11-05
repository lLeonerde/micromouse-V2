#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#define MAZE_SIZE 8  // Define o tamanho do labirinto como 8x8 células
#define MAX_PATH_LENGTH (MAZE_SIZE * MAZE_SIZE)  // Máximo de células que podem fazer parte do caminho
#define INF 99999  // Define um valor de "infinito" para inicializar as distâncias

// Estrutura que armazena coordenadas (x, y) e a direção do robô
typedef struct {
    int x;  // Posição no eixo X
    int y;  // Posição no eixo Y
    int direction;  // Direção atual do robô: 0=Norte, 1=Leste, 2=Sul, 3=Oeste
} Coord;

// Estrutura que representa cada célula do labirinto
typedef struct {
    bool visited;  // Indica se a célula já foi visitada
    bool wallNorth;  // Indica se há uma parede ao norte
    bool wallSouth;  // Indica se há uma parede ao sul
    bool wallEast;   // Indica se há uma parede a leste
    bool wallWest;   // Indica se há uma parede a oeste
    int distance;    // Distância da célula até o alvo, usada para o cálculo do caminho
} Cell;

// Estrutura de um nó para a fila encadeada, usada no BFS
typedef struct Node {
    Coord coord;  // Coordenada do nó
    struct Node *next;  // Próximo nó na fila
} Node;

// Estrutura da fila encadeada
typedef struct {
    Node *front;  // Posição inicial da fila
    Node *rear;   // Posição final da fila
    int size;     // Tamanho atual da fila
} Queue;

// Estrutura que armazena o estado do labirinto e do robô
typedef struct {
    Cell maze[MAZE_SIZE][MAZE_SIZE];  // Matriz representando o labirinto
    Coord path[MAX_PATH_LENGTH];      // Array que armazena o caminho percorrido
    int pathIndex;                    // Índice do caminho atual
    Coord current;                    // Posição e direção atual do robô
    bool targetReached;               // Indica se o robô chegou ao alvo (centro do labirinto)
} MazeState;

// Funções de manipulação da fila

// Inicializa uma fila vazia
void initQueue(Queue *q) {
    q->front = q->rear = NULL;  // Define a frente e a traseira da fila como nulas
    q->size = 0;  // Inicializa o tamanho da fila como zero
}

// Adiciona uma nova coordenada à fila
void enqueue(Queue *q, Coord coord) {
    Node *temp = (Node *)malloc(sizeof(Node));  // Aloca memória para um novo nó
    temp->coord = coord;  // Define a coordenada do novo nó
    temp->next = NULL;  // O próximo nó será nulo (último da fila)
    
    if (q->rear == NULL) {  // Se a fila estiver vazia
        q->front = q->rear = temp;  // O novo nó será tanto a frente quanto a traseira
    } else {
        q->rear->next = temp;  // Adiciona o novo nó ao final da fila
        q->rear = temp;  // Atualiza a traseira da fila
    }
    q->size++;  // Incrementa o tamanho da fila
}

// Remove e retorna a coordenada da frente da fila
Coord dequeue(Queue *q) {
    if (q->front == NULL) {  // Se a fila estiver vazia, retorna uma coordenada inválida
        Coord invalid = {-1, -1, -1};  // Coordenada inválida
        return invalid;
    }
    
    Node *temp = q->front;  // Armazena o nó da frente para removê-lo
    Coord coord = temp->coord;  // Recupera a coordenada da frente
    q->front = q->front->next;  // Move a frente da fila para o próximo nó
    if (q->front == NULL) q->rear = NULL;  // Se a fila ficar vazia, ajusta a traseira
    free(temp);  // Libera a memória do nó removido
    q->size--;  // Decrementa o tamanho da fila
    return coord;  // Retorna a coordenada removida
}

// Inicializa o estado do labirinto
void initMazeState(MazeState *state) {
    // Limpa a memória do labirinto e inicializa com valores padrões
    memset(state->maze, 0, sizeof(state->maze));
    state->pathIndex = 0;  // Inicializa o índice do caminho
    state->current = (Coord){0, 0, 0};  // Define a posição inicial (0,0) e a direção como norte (0)
    state->targetReached = false;  // Inicializa o estado de alvo não alcançado
    
    // Inicializa todas as distâncias das células com "infinito"
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            state->maze[i][j].distance = INF;
        }
    }
    
    // Define a distância inicial no centro do labirinto como zero (alvo)
    state->maze[MAZE_SIZE/2][MAZE_SIZE/2].distance = 0;
}

// Verifica se uma coordenada está dentro dos limites do labirinto
bool isValidCoord(int x, int y) {
    return x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE;  // Coordenadas válidas entre 0 e MAZE_SIZE-1
}

// Atualiza as distâncias no labirinto usando BFS (Busca em Largura)
void updateDistances(MazeState *state) {
    Queue q;
    initQueue(&q);
    
    // Inicia a BFS a partir do centro do labirinto
    Coord center = {MAZE_SIZE/2, MAZE_SIZE/2, 0};
    enqueue(&q, center);  // Enfileira a célula central
    
    while (q.size > 0) {  // Enquanto houver elementos na fila
        Coord current = dequeue(&q);  // Desenfileira a célula atual
        Cell *currentCell = &state->maze[current.x][current.y];  // Obtém a célula atual no labirinto
        
        // Arrays que definem os deslocamentos em X e Y para cada direção (Norte, Leste, Sul, Oeste)
        const int dx[] = {0, 1, 0, -1};
        const int dy[] = {-1, 0, 1, 0};
        
        // Itera sobre as quatro direções
        for (int dir = 0; dir < 4; dir++) {
            int newX = current.x + dx[dir];  // Calcula a nova coordenada X
            int newY = current.y + dy[dir];  // Calcula a nova coordenada Y
            
            if (!isValidCoord(newX, newY)) continue;  // Verifica se a nova coordenada é válida
            
            Cell *nextCell = &state->maze[newX][newY];  // Obtém a próxima célula
            bool hasWall = false;  // Variável para verificar se há uma parede entre as células
            
            // Verifica a presença de paredes com base na direção
            switch(dir) {
                case 0: hasWall = currentCell->wallNorth; break;  // Norte
                case 1: hasWall = currentCell->wallEast; break;   // Leste
                case 2: hasWall = currentCell->wallSouth; break;  // Sul
                case 3: hasWall = currentCell->wallWest; break;   // Oeste
            }
            
            // Se não há parede e a nova célula tem uma distância maior, atualiza a distância e enfileira
            if (!hasWall && nextCell->distance > currentCell->distance + 1) {
                nextCell->distance = currentCell->distance + 1;
                enqueue(&q, (Coord){newX, newY, dir});  // Enfileira a próxima célula
            }
        }
    }
}

// Função que detecta paredes ao redor do robô com base nos sensores
void detectWalls(MazeState *state) {
    Cell *currentCell = &state->maze[state->current.x][state->current.y];  // Obtém a célula atual
    
    // Simulação da leitura de sensores (substituir com leituras reais)
    bool wallFront = false;  // Leitura do sensor frontal
    bool wallRight = false;  // Leitura do sensor direito
    bool wallLeft = false;   // Leitura do sensor esquerdo
    
    // Atualiza a informação das paredes com base na direção atual do robô
    switch(state->current.direction) {
        case 0: // Norte
            currentCell->wallNorth = wallFront;
            currentCell->wallEast = wallRight;
            currentCell->wallWest = wallLeft;
            break;
        case 1: // Leste
            currentCell->wallEast = wallFront;
            currentCell->wallSouth = wallRight;
            currentCell->wallNorth = wallLeft;
            break;
        case 2: // Sul
            currentCell->wallSouth = wallFront;
            currentCell->wallWest = wallRight;
            currentCell->wallEast = wallLeft;
            break;
        case 3: // Oeste
            currentCell->wallWest = wallFront;
            currentCell->wallNorth = wallRight;
            currentCell->wallSouth = wallLeft;
            break;
    }
    
    currentCell->visited = true;  // Marca a célula como visitada
}

// Move o robô para a próxima célula com base nas distâncias
void moveRobot(MazeState *state) {
    int minDist = INF;  // Variável para armazenar a menor distância encontrada
    Coord nextMove = state->current;  // Coordenada do próximo movimento
    
    // Arrays que definem os deslocamentos em X e Y para cada direção
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};
    
    // Itera sobre as quatro direções para encontrar a célula vizinha com menor distância
    for (int dir = 0; dir < 4; dir++) {
        int newX = state->current.x + dx[dir];  // Nova coordenada X
        int newY = state->current.y + dy[dir];  // Nova coordenada Y
        
        if (!isValidCoord(newX, newY)) continue;  // Verifica se a coordenada é válida
        
        Cell *nextCell = &state->maze[newX][newY];  // Obtém a próxima célula
        Cell *currentCell = &state->maze[state->current.x][state->current.y];  // Obtém a célula atual
        
        bool hasWall = false;  // Verifica a presença de paredes
        switch(dir) {
            case 0: hasWall = currentCell->wallNorth; break;  // Norte
            case 1: hasWall = currentCell->wallEast; break;   // Leste
            case 2: hasWall = currentCell->wallSouth; break;  // Sul
            case 3: hasWall = currentCell->wallWest; break;   // Oeste
        }
        
        // Se não há parede, a célula não foi visitada e tem menor distância, atualiza o movimento
        if (!hasWall && !nextCell->visited && nextCell->distance < minDist) {
            minDist = nextCell->distance;
            nextMove = (Coord){newX, newY, dir};  // Próximo movimento do robô
        }
    }
    
    // Atualiza a posição atual do robô para o próximo movimento
    state->current = nextMove;
    state->path[state->pathIndex++] = nextMove;  // Adiciona o movimento ao caminho
    
    // Aqui deve ser implementado o movimento físico do robô
    printf("Movendo para (%d, %d) direção %d\n", nextMove.x, nextMove.y, nextMove.direction);
}

// Função para mover o robô de volta ao ponto inicial de forma otimizada
void returnToStart(MazeState *state) {
    // Enquanto o robô não retornar à célula inicial (0,0)
    while (state->current.x != 0 || state->current.y != 0) {
        int minDist = INF;  // Variável para armazenar a menor distância encontrada
        Coord nextMove = state->current;  // Coordenada do próximo movimento
        
        // Arrays que definem os deslocamentos em X e Y para cada direção (Norte, Leste, Sul, Oeste)
        const int dx[] = {0, 1, 0, -1};
        const int dy[] = {-1, 0, 1, 0};
        
        // Itera sobre as quatro direções para encontrar a célula vizinha com menor distância
        for (int dir = 0; dir < 4; dir++) {
            int newX = state->current.x + dx[dir];  // Nova coordenada X
            int newY = state->current.y + dy[dir];  // Nova coordenada Y
            
            if (!isValidCoord(newX, newY)) continue;  // Verifica se a coordenada é válida
            
            Cell *nextCell = &state->maze[newX][newY];  // Obtém a próxima célula
            Cell *currentCell = &state->maze[state->current.x][state->current.y];  // Obtém a célula atual
            
            bool hasWall = false;  // Verifica a presença de paredes
            switch(dir) {
                case 0: hasWall = currentCell->wallNorth; break;  // Norte
                case 1: hasWall = currentCell->wallEast; break;   // Leste
                case 2: hasWall = currentCell->wallSouth; break;  // Sul
                case 3: hasWall = currentCell->wallWest; break;   // Oeste
            }
            
            // Se não há parede e a nova célula tem menor distância, atualiza o movimento
            if (!hasWall && nextCell->distance < minDist) {
                minDist = nextCell->distance;
                nextMove = (Coord){newX, newY, dir};  // Próximo movimento do robô
            }
        }
        
        // Atualiza a posição atual do robô para o próximo movimento
        state->current = nextMove;
        state->path[state->pathIndex++] = nextMove;  // Adiciona o movimento ao caminho
        
        // Aqui deve ser implementado o movimento físico do robô
        printf("Movendo de volta para (%d, %d) direção %d\n", nextMove.x, nextMove.y, nextMove.direction);
    }
    
    printf("O robô retornou à posição inicial (0,0).\n");
}


// Função principal para exploração do labirinto
void exploreMaze(void) {
    MazeState state;
    initMazeState(&state);  // Inicializa o estado do labirinto
    
    // Explora o labirinto até o alvo ser alcançado
    while (!state.targetReached) {
        detectWalls(&state);  // Detecta paredes ao redor do robô
        updateDistances(&state);  // Atualiza as distâncias no labirinto
        moveRobot(&state);  // Move o robô para a próxima célula
        
        // Verifica se o robô chegou ao centro do labirinto
        if (state.current.x == MAZE_SIZE/2 && state.current.y == MAZE_SIZE/2) {
            state.targetReached = true;  // Marca o alvo como alcançado
        }
        // Após o robô alcançar o centro, ele retorna ao início
        if (state.targetReached) {
            printf("O robô alcançou o centro, agora retornando ao ponto inicial.\n");
            returnToStart(&state);  // Retorna ao início de forma otimizada
        }

    }
    
    // Imprime o caminho percorrido pelo robô
    printf("Caminho encontrado:\n");
    for (int i = 0; i < state.pathIndex; i++) {
        printf("(%d, %d) dir=%d\n", state.path[i].x, state.path[i].y, state.path[i].direction);
    }
}

// Função principal do programa
void app_main(void) {
    exploreMaze();  // Inicia a exploração do labirinto
}
