Para usar o udatmo descomente a seguinte linha no behavior_selector.cpp:

#define USE_DATMO_GOAL

Para imprimir os dados da detecção de objetos descomente a linha no behavior_selector.cpp:

#define PRINT_UDATMO_LOG

Para simular objetos moveis descomente as seguntes linhas no behavior_selector_main.cpp:

#define SIMULATE_MOVING_OBSTACLE			//para simular um objeto à frente do carro
#define SIMULATE_LATERAL_MOVING_OBSTACLE	//para simular um objeto que ultrapassa o carro