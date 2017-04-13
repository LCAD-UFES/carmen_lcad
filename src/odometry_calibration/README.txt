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

--------guarapari-split2--------------- verificar
bias v: 0.976859 0.000000 bias phi: 0.500000 -0.001350 initial angle: 2.619321
Fitness (MSE): -272.956184
Fitness (SQRT(MSE)): 16.521386
---------------------------------------

--------guarapari-split3---------------
bias v: 0.989939 0.000000 bias phi: 0.976104 -0.002454 initial angle: 1.277144
Fitness (MSE): -61.158948
Fitness (SQRT(MSE)): 7.820419
---------------------------------------

--------guarapari-split4---------------
bias v: 0.986904 -0.000000 bias phi: 0.999119 -0.002401 initial angle: 2.953716
Fitness (MSE): -40.331249
Fitness (SQRT(MSE)): 6.350689
--------------------------------------

--------guarapari-completo------------------
bias v: 0.901984 -0.000000 bias phi: 0.500000 -0.001356 initial angle: -1.147907
Fitness (MSE): -1767.096363
Fitness (SQRT(MSE)): 42.036845
---------------------------

---------praca------------
bias v: 1.003211 0.000000 bias phi: 1.093834 -0.003157 initial angle: -2.754635
Fitness (MSE): -1.029459
Fitness (SQRT(MSE)): 1.014623
--------------------------

---------ponte------------1228
bias v: 0.995581 -0.000000 bias phi: 1.008126 -0.003212 initial angle: 0.730014
Fitness (MSE): -14.278659
Fitness (SQRT(MSE)): 3.778711
--------------------------

---------ponte------------1208
bias v: 0.973102 -0.000000 bias phi: 0.774490 -0.002003 initial angle: -2.711343
Fitness (MSE): -37.309864
Fitness (SQRT(MSE)): 6.108180

bias v: 0.973049 -0.000000 bias phi: 0.774100 -0.002003 initial angle: -2.710927
Fitness (MSE): -37.236513
Fitness (SQRT(MSE)): 6.102173


bias v: 0.973049 -0.000000 bias phi: 0.774100 -0.002003 initial angle: -2.710927
Fitness (MSE): -37.236513
Fitness (SQRT(MSE)): 6.102173

--------------------
bias v: 1.021018 -0.000000 bias phi: 1.109900 -0.003390 initial angle: 0.797249
Fitness (MSE): -12.613491
Fitness (SQRT(MSE)): 3.551548


bias v: 0.941128 -0.000000 bias phi: 0.893987 -0.001730 initial angle: -0.150506
Fitness (MSE): -70.501351
Fitness (SQRT(MSE)): 8.396508
thomas@thomas-XPS:~/carmen_lcad/src/odometry_calibration$ 


bias v: 0.977105 0.000000 bias phi: 0.990515 -0.001933 initial angle: 0.288941
Fitness (MSE): -60.637982
Fitness (SQRT(MSE)): 7.787039


bias v: 0.974839 -0.000000 bias phi: 0.982536 -0.001915 initial angle: 0.253621  
Fitness (MSE): -60.024483
Fitness (SQRT(MSE)): 7.747547


bias v: 0.967690 -0.000000 bias phi: 0.961846 -0.001870 initial angle: 0.157233
Fitness (MSE): -59.654002
Fitness (SQRT(MSE)): 7.723600



bias v: 0.970045 -0.000000 bias phi: 0.968945 -0.001886 initial angle: 0.189910
Fitness (MSE): -59.605144
Fitness (SQRT(MSE)): 7.720437 sem...


bias v: 0.969521 0.000000 bias phi: 0.967416 -0.001882 initial angle: 0.182882 4
Fitness (MSE): -59.601204
Fitness (SQRT(MSE)): 7.720182

