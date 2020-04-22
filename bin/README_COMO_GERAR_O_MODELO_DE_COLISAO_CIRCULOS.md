MODELO DE COLISÃO (ford_escape_collision_file.txt):
Neste arquivo o robô é definido por um conjunto de círculos.
Os círculos serao usados para checagem de colisão usando o mapa de distância para obstáculos.
O robô esta em situação de colisão se na posição de um dos cículos que representa o robô existe um obistáculo a uma distância menor ou igual ao raio do círculo.
X Y são relativos ao meio do eixo traseiro.
O arquivo do modelo de colisão tem o seguinte formato:

Número de círculos que definem o robo
(Número de níveis de mapas) -1 (se estiver usando apenas um nível de mapa este valor deve ser 0)
X1 Y1 R1 (Nível do mapa em que se encontra o círculo)
X2 Y2 R2 (Nível do mapa em que se encontra o círculo)
X3 Y3 R3 (Nível do mapa em que se encontra o círculo)
Xn Yn Rn (Nível do mapa em que se encontra o círculo)

Exemplo do arquivo do modelo de colisão da IARA:
5
0
-0.76 0.0 1.0 0
0.207 0.0 1.0 0
1.174 0.0 1.0 0
2.141 0.0 1.0 0
3.108 0.0 1.0 0


MODELO DE VISUALIZAÇÃO 2D:
Este arquivo define o poligono que desenha o robô no navigator_gui
X Y são relativos ao meio do eixo traseiro.
A ordem dos pontos é importanete para a correta visualização.
O arquivo do modelo de visualização tem o seguinte formato:

displacement 
Número de pontos que definem o polígono
X1 Y1
X2 Y2
X3 Y3
Xn Yn

Exemplo do arquivo do modelo de visualização da IARA:
3.475
4
-0.96 0.903
-0.96 -0.903
3.465 -0.903
3.465 0.903
