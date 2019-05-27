Como calibrar a odometria:

1. compile o modulo odometry_calibration
2. Vá para o diretório bin
 cd $CARMEN_HOME/bin
2. Execute o programa:
 ./calibrate_bias_from_log --gps_to_use 2 --use_non_linear_phi 0 -i 300 /dados/log_estacionameno_ambiental_20190503-ford-fusion-2.txt ../src/carmen-ford-fusion.ini caco.txt poses.txt poses_opt.txt

	Notas:
		- Para saber os parametros do programa execute ele sem nenhum parametro.
		- No uso do dia-a-dia, rode o programa mais de uma vez e observe qual a melhor calibração no gráfico final (o método é probabilístico).
		- Você pode delimitar o tempo inicial e final a ser considerado no log verificando o tempo no fim de cada linha do log e 
		  o número da linha do tempo inicial e final. Use as flags -l <linha inicial>  e -m <numero de linhas a partir da linha inicial>.

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

4. Voce pode usar o poses_opt.txt para fazer mapas sem graphslam! Basta usa-lo como se fosse a saida do graphslam para o 
log. Para isso, no process de fazer mapas, basta usar a linha do graphslam_publish como abaixo:
 Publish_poses		graphslam	1		0			./graphslam_publish ../src/odometry_calibration/poses_opt.txt

