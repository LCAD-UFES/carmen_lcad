- O rddf representa a rota ideal desejada, R. Neste planejador, ela (R) deve ser levada para o frenet frame, FF, definido em coordenadas (s, d). Pontos (x, y) de R 
sao levados para pontos (s, d) de FF usando R como base. O planejador deve gerar planos (path plans) ao longo R para afastamentos de distancias fixas na dimensao d de sua representacao FF. 
Podemos pensar, inicialmente, em 30 rotas paralelas aa original: 15 aa direita e 15 aa esquerda. Estas 30 rotas paralelas devem cobrir duas lanes aa direita e duas aa esquerda 
(cerca de 6 metros para cada lado, 3m por lane: http://www.ebanataw.com.br/trafegando/faixas.htm), com afastamento entre elas na dimensao d igual a aproximadamente 0.4m.
- O planejador deve gerar, neste cenario, 31 planos: 30 que levem a cada uma das 30 rotas paralelas a R e um que leve para R. O behavior selector deve escolher o melhor plano, 
um goal neste plano e a velocidade do goal. O restante do sistema funciona como previamente.
- Os planos devem partir da posicao aproximada atual do carro, PA, que eh a posicao no plano atualmente seguido (um dos 31) mais proxima possivel da globalpos. 
Alem disso, devem possuir continuidade temporal; isto eh, devem se conectar suavemente (incluir parte?) ao plano do time step anterior
(ver imagem em https://www.researchgate.net/figure/Optimal-trajectory-thick-gray-with-end-point-on-the-continuous-terminal-manifold_fig5_254098780).
- Os planos podem ser computados a partir de PA por meio de um spline na dimensao d de FF, que tenha a dimensao s de FF como base (o spline eh em (s, d)). Para garantir a continuidade temporal, 
a derivada de d com relacao a s, dd/ds, do primeiro ponto, p0(t), do plano correntemente sendo seguido no tempo t, P(t), deve ser a mesma para todos os 31 planos e 
coincidir com a dd/ds do ponto px(t-1), correspondente a p0(t) do plano corrente no time step anterior, P(t-1), entre ciclos de planejamento para cada um dos 31 planos. 
Isto eh, a dd/ds do primeiro ponto, p0(t) = (s0(t), d0(t)), de um plano do time step atual, P(t), deve ser igual aa dd/ds da projecao de p0(t) na versao anterior de P(t), ou P(t-1). 
Ou seja, a dd/ds de p0(t) deve ser igual aa dd/ds do ponto px(t-1), onde a posicao de px(t-1) no mundo eh a mais proxima possivel de PA.
- Note que, alem da dd/ds, os valores de s e d de p0(t) de todos os 31 planos eh igual, mas eles divergem a partir de p0(t) para alcancar as 31 rotas paralelas no horizonte de tempo, 
T, e distancia na dimensao s de FF, D(s), considerados pelo planejador.
- O plano correntemente sendo seguido no tempo t, P(t), pode representar uma continuidade incompleta do afastamente de R na direcao de uma das rotas paralelas a R.
Para garantir continuidade, o plano P(t) deve ter a mesma forma do plano anterior, P(t-1), a menos que o plano seja mudado em t. Assim, o plano anterior, P(t-1), deve ser apenas
extendido para se gerar P(t). Os planos paralelos a P(t) devem ser, no entanto, recomputados completamente a partir de PA com as restricoes mencionadas acima.
- No momento inicial, os 31 planos sao gerados a partir da globalpos, isto eh, PA = globalpos. Depois do instante inicial, o plano sendo seguido em P(t-1) eh
extendido a partir de PA (p0 de P(t) eh igual a PA) para se gerar P(t), e os demais sao computados a partir de PA.
- Os planos podem alcancar suas rotas paralelas a R antes do fim de T (em T/2, por exemplo) caso se deseje manobras mais agressivas.
- O horizonte de tempo T considerado pode ser aproximandamente igual a 5 segundos, equanto que o horizonte de distancia D(s) vai de um minimo para velocidades baixas ou zero 
(13m? Um multiplo do tamanho do carro?), ate a distancia alcancavel dentro de T na velocidade e aceleracao atuais (no momento t).
- Um conjunto de valores de T e D(s) podem ser considerados para a geracao de multiplos de 31 planos. Estes planos extras permitiriam mais opcoes ao behavior selector.
- Os valores maximos de dd/ds vao variar de acordo com os valores de T e D(s) considerados e alguns planos podem ser inviaveis e devem ser descartados por ultrapassar os limites de 
velocidade de variacao de phi, de aceleracao do carro, de forca centrifuga, ou de jerk apos a conversao dos pontos (s, d) do plano para (x, y), onde estes limites podem ser facilmente verificados.
- Um plano especial pode ser tambem demandado pelo behavior selector e computado pelo planejador para permitir manobras de precisao, como as de estacionamento e passagem
por espacos estreitos (cancela, por exemplo). Este plano especial seria especificado como um afastamento especifico na dimensao d do FF da R, e uma distancia D(s).


Para rodar o frenet_path_planner junto com o simulador udacity:
- Compile o codigo de interface entre o simulador e o carmen_lcad
 cd PathPlanning
 mkdir build
 cd build
 cmake ..
 make

- Rode o central e o process apropriado:
Abra um terminal e rode o cetral
 cd carmen_lcad/bin
 ./central

Abra um terminal e rode o process
 ./proccontrol process-navigate-udacity.ini

- Rode a interface com o simulador da Udacity e o simulador

Abra um terminal e rode:
 cd src/frenet_path_planner/PathPlanning/build
 ./path_planning

Abra um terminal e rode:
 cd src/frenet_path_planner/term3_sim_linux
 ./term3_sim.x86_64

Escolha a qualidade da simulacao e clique Select


TODO:
Checar o uso de carmen_obstacle_avoider_compute_car_distance_to_closest_obstacles() em compute_proximity_to_obstacles_using_distance_map() e path_has_collision_or_phi_exceeded()

