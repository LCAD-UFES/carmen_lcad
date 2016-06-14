como calibrar a odometria:

1. compile o modulo odometry_calibration
2. Execute o central
3. Rode ./proccontrol process-volta_da_ufes_playback_viewer_3D.ini {atualize-o antes com o nome de seu log e NÃO aperte play}
4. Execute o programa "./grab_data_for_calibration out.txt" 
5. Faca playback de seu log ate o final {aperte play}
6. Mate o programa "./grab_data_for_calibration out.txt" e o proccontrol
7. Execute o programa "./calibrate_bias out.txt > debug.txt" 10 vezes e escolha o melhor resultado ("Fitness (MSE)" mais proximo de zero)
	Nota: Os bias obtidos estao no seguinte formato:
	- bias v: <bias multiplicativo> <bias aditivo> bias phi: <bias multiplicativo> <bias aditivo (em rad)> initial angle: <angulo em rad>
8. Para visualizar o resultado, use o programa gnuplot:
	> gnuplot
	> $> plot './debug.txt' using 1:2 title 'calibrated odometry' with lines
	> $> replot './debug.txt' using 3:4 title 'GPS' with lines
	> $> replot './debug.txt' using 5:6 title 'raw odometry' with lines
9. Altere as variaveis abaixo no carmen-ford-escape.ini de acordo com os resultados mencionados acima:
robot_phi_multiplier				1.109900
robot_phi_bias					-0.003390
robot_v_multiplier				1.021018

bias v: 1.021018 -0.000000 bias phi: 1.109900 -0.003390 initial angle: 0.797249
Fitness (MSE): -12.613491
Fitness (SQRT(MSE)): 3.551548
