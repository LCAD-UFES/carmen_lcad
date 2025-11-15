# FastSLAM

O FastSLAM (implementação atual é, na verdade, o FastSLAM 2.0 para mapas de grid) é um algoritmo de SLAM online para geração de mapas, que considera que o último
estado do robôs contém toda a informação sobre a pose do robô e o mapa (assunção de Markov), ao contrário do GraphSLAM, que é um algoritmo de full SLAM offline, 
que considera todos os estados do log (mas não os do Velodyne, mas apenas seu timestamp...) simultanemente para fazer o mapa e saber onde o robô estava no mapa.

Assim, o estado inicial é muito importante para um bom mapa e o robô deve estar parado por pelo alguns segundos (6 é um bom mínimo) e com bom GPS para que o
mapa fique bom e bem ancorado no mundo. A orientação do GPS é fundamental. Para GPSs sem orientação, invocar o FasSLAM com a flag que indica o ângulo inicial 
(initial_angle, ver abaixo).

Além de um bom estado inicial, a odometria deve estar calibrada, assim como a posição dos sensores (no carmen ini), especialmente a posição da sensor_board,
do Velodyne (melhor colocar o ângulo de pitch e yaw no Velodyne, e não na sensor_board) e do GPS.


## Como compilar

Basta ir na pasta src/fastslam de executar make
 cd src/fastslam
 make


## Como usar

O FastSLAM é uma mistura do localize_ackerman e do mapper e tem interface similar ao mapper. Para usá-lo, execute o process process-fastslam.ini 
com o seu log de interesse e, no diretório bin, execute o comando:
 ./fastslam -map_path ../data/mapper_teste2 -calibration_file calibration_table.txt

No playback control (interface do logger), clique no botão Play para iniciar o mapeamento. Pode ser necessário clicar mais 
de uma vez no início da execução/construção do mapa, já que o FastSLAM emprega as mesmas mensagens do playback control para ajustar o fluxo de mensagens 
de modo a otimizar o tempo de execução sem perda de mensagens. Se você quiser pausar a contrução do mapa (importante no fechamento de loops), 
clique várias vezes no botão de Stop até parar a execucão do log (o FastSLAM usa as mensagens do playback control; portanto, há
uma competição entre o usuário e o FastSLAM sobre o fluxo do log).

Com o comando acima (./fastslam ...), o FastSLAM vai criar, depois do ctrl+c final, o mapa no diretório ../data/mapper_teste2
Note que o FastSLAM não cria o mapa de remission e outros mapas associados, mas apenas o mapa de occupancy.

Se o log estiver parado, é necessário teclar ctrl+c duas vezes para interromper o programa e salvar o mapa final.

O FastSLAM publica a globalpos. Assim, você pode gerar o RDDF do log ao mesmo tempo em que faz o mapa. Basta rodar o rddf_build ao mesmo tempo em que o
FastSLAM está fazendo o mapa.


## Parâmetros

O FastSLAM possui parâmetros específicos no carmen ini:
* fastslam_num_particles				1
* fastslam_num_particles_improved		300
* fastslam_log_playback_speed			1.0
* fastslam_log_time_between_likelihood_maps_update 0.5

* fastslam_localize_lmap_std									2.40
* fastslam_localize_yaw_uncertainty_due_to_grid_resolution	0.107 # em graus
* fastslam_localize_xy_uncertainty_due_to_grid_resolution	0.15  # em metros
* fastslam_localize_particles_normalize_factor			0.05
* fastslam_gps_correction_factor					1.0   # 0.0 -> sem gps correction, 1.0 -> gps correction forte, >1 -> gps correction maior que a importancia do mapa

O primeiro determina quantas partículas de mapa serão usadas. O ideal seriam 25, mas, na versão atual do FastSLAM rodando com computadores típicos do 
LCAD de 2022, manter 25 mapas gasta muita memória e requer muito poder de processamento. 

O segundo parâmetro determina quantas partículas de localização teremos por mapa. O ideal seriam 300, mas isso torna o FastSLAM muito lento
com computadores atuais também se fastslam_num_particles é grande (o número de processos de localização é igual a 
fastslam_num_particles x fastslam_num_particles_improved).

Note que todos os parâmetros do localize_ackerman e do mapper influenciam o FastSLAM, exceto os que o FastSLAM sobrescreve:

* fastslam_localize_lmap_std									2.40
* fastslam_localize_yaw_uncertainty_due_to_grid_resolution	0.107 # em graus
* fastslam_localize_xy_uncertainty_due_to_grid_resolution	0.15  # em metros
* fastslam_localize_particles_normalize_factor			0.02

O último parâmetro, fastslam_gps_correction_factor, indica o quanto o GPS é considerado no processo de fazer o mapa com o FastSLAM.
Se o GPS estiver bom, use um valor próximo de 0.5. Se estiver ruim, use 0. Para mapear com GPS apenas
use um valor grande (10.0, por exemplo).

O FastSLAM precisa saber a orientação inicial com precisão. Caso ela não esteja boa (oriunda de IMU apenas, por exemplo)
ou não esteja disponível, use o parâmetro de linha `initial_theta` para informar o ângulo inicial do robô
manualmente:
` ./fastslam -map_path ../data/mapper_teste2 -calibration_file calibration_table.txt -initial_theta 0.66`

O ângulo deve ser informado em radianos.


## Merge de mapas

Para fazer merge de mapas, basta rodar um log e fazer um mapa base e, depois, rodar o outro log e fazer o novo mapa em cima 
do anterior. É importante que os dois logs possuam um bom sinal de GPS no ponto onde o merge se inicia. O parâmetro fastslam_gps_correction_factor
deve ser igual a zero quando da execução do segundo log (o GPS vai ser usado apenas para a localização inicial).


## Mapas com sinal de GPS ruim

O FastSLAM pode fazer mapas sem nenhum GPS, usando apenas a odometria e os dados de Velodyne. Pode fazer mapas, também, em regiões
que alternam sinal de GPS bom e sinal de GPS ruim (entrada e saídas de galpões). Para o mapa ficar bem ancorado no mundo, o início do
mapeamento deve ser feito num ponto de bom sinal de GPS.


## Parâmetros de linha de comando

* `-use_gps on/off`: considera ou não o sinal do GPS.

* `-initial_x`, `-initial_y`, `-initial_theta`: poses iniciais que bypassam as poses iniciais obtidas pela fused odometry (se `-use_gps off`) ou do GPS (se `-use_gps on`)

* `-save_globalpos_file <filename>`: salva as poses globais no devido arquivo, com as colunas

| globalpos_x | globalpos_y | globalpos_z | globalpos_yaw | gps_x | gps_y | latitude | longitude | gps_quality | v | phi | timestamp |
|---|---|---|---|---|---|---|---|---|---|---|---|


DICA: Se a odometria estiver ruim e o GPS aceitável, faça:
```
fastslam_gps_correction_factor = 100000.0

// E espalhe bem as partículas para que ele siga o GPS
localize_ackerman_phi_noise_phi = 100000.0 ou localize_ackerman_phi_noise_velocity = 10000.0

// phi_noise_phi é um ruído adicionado ao phi a partir do valor de phi, e phi_noise_velocity um ruído adicionado ao phi a partir da velocity. Se phi é zero, phi_noise_phi não tem efeito.
```

E smooth o GPS, para que ele não fique quicando

* `./gps_xyz -kalman on`
* `./fastslam -use_gps on`
