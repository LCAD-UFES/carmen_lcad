Como calibrar a odometria:

1. compile o modulo odometry_calibration
2. Execute o programa:
 ./calibrate_bias_from_log --gps_to_use 2 --use_non_linear_phi 0 /dados/log_estacionameno_ambiental_20190503-ford-fusion-2.txt ../carmen-ford-fusion.ini caco.txt poses.txt

	Notas:
		- Para saber os parametros do programa execute ele sem nenhum parametro
		- Rode mais de uma vez e observe qual a melhor calibração no gráfico final

3. Altere as variaveis abaixo no carmen-ford-escape.ini de acordo com os Resultados do calibrate_bias_from_log:
robot_phi_multiplier				1.056087
robot_phi_bias					1.065384
robot_v_multiplier				0.003477

Resultados:
...
v (multiplier bias): (1.056087 0.000000),  phi (multiplier bias): (1.065384 0.003477),  Initial Angle: 2.449064,  k1: 0.000000,  k2: 0.000000
Fitness (MSE): -2.400199
Fitness (SQRT(MSE)): 1.549258
Press a key to finish...

