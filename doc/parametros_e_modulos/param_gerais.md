# Descrição

Esse arquivo contém parâmetros que não estão atrelados a um módulo em específico, sendo utilizados por múltiplos módulos

# Parâmetros

## Lidar

No carmen existem três "classificações" para um lidar, e cada categoria tem espaço para ter parâmetros de múltiplos lidares ao mesmo tempo no arquivo de parâmetro, pro exemplo, os velodynes podem ir até 9 em alguns casos:
- **VelodyneX:** A classificação mais específica de lidar, são lidares do tipo velodyne cujos dados se organizam em mensagens com *scans*. Esses lidares tem um conjunto de laser alinhados na horizontal mas com ângulos verticais diferentes, geralmente 16 ou 32 raios, e giram ao redor de um eixo, disparando os raios e medindo o tempo que demoram para voltar, calculando a distância e criando um grid 3D com os pontos refletidos.
Cada velodyne tem um conjunto padrão de parâmetros que estão disponíveis para múltiplos módulos, e ocasionalmente tem parâmetros específicos de módulos. Esse arquivo só conterá os parâmetros gerais, mas muitas vezes módulos como *localize_ackerman* terão parâmetros a mais para identificar esse lidar. Para cada velodyne, o parâmetro que o identifica é velodyneX, em que X é seu número (Ex: velodyne0).
- **LidarX:** Vale tudo que vale para o velodyneX, mas as mensagens usadas para comunicar são mais flexíveis, e no geral os parâmetros também.
- **xyz_lidarX:** Semelhantes à classificação anterior, mas ao invés de organizar os dados em formato de *scans*, eles são enviados em um conjunto de pontos em coordenadas cartesianas. OBS: no tempo em que foi feito essa documentação (06/06/2025) esses lidares só estão funcionando no módulo Viewer3D, e não conseguem ser usados para gerar localização em mapeamento pois não tem código implementado para isso.

### **velodyneX_model**, **lidarX_model** e **xyz_lidarX_model**

Identifica o modelo do lidar em questão (Ex: Ouster, VLP16 e outros).

### **lidarX_port**, **lidarX_imu_port**, **xyz_lidarX_port** e **xyz_lidarX_imu_port**

Portas usadas para comunicação com o lidar (geralmente configurável no painel de controle do lidar, colocando o IP do lidar no navegador).

### **lidarX_ip** e **xyz_lidarX_ip**

IP do lidar (geralmente comunica via ethernet, pode usar nmap para achar).

### **lidarX_shot_size** 

Número de *pontos* em um *scan* (quantidade de pontos gerados pelos laser em um *scan*, é igual ao número de laser do lidar).

### **lidarX_min_sensing** 

Não sei, quando descobrir atualize essa documentação.

### **lidarX_max_sensing** 

Não sei, quando descobrir atualize essa documentação.

### **lidarX_range_division_factor** 

Não sei, quando descobrir atualize essa documentação.

### **lidarX_max_range** e **xyz_lidarX_max_range**

Máximo alcance do lidar, por exemplo, se esse parâmetro é 70.0 ele irá desprezar pontos além de 70m de distância (OBS: Não sei se esse parâmetro funciona, pois outros módulos como o mapper tem versões próprias dele).

### **lidarX_time_between_shots** e **velodyneX_time_spent_by_each_scan**

Tempo que o lidar leva entre cada shot, é util caso sejam enviados múltiplos shots na mensagem e tenha apenas o timestamp do primeiro shot.

### **lidarX_x**, **xyz_lidarX_x** e **velodyneX_x** 

Coordenada X do lidar em relação ao robô (geralmente sensorboard, depende de **sensor_reference**).

### **lidarX_y**, **xyz_lidarX_y** e **velodyneX_y** 

Coordenada Y do lidar em relação ao robô (geralmente sensorboard, depende de **sensor_reference**).

### **lidarX_z**, **xyz_lidarX_z** e **velodyneX_z** 

Coordenada Z do lidar em relação ao robô (geralmente sensorboard, depende de **sensor_reference**).

### **lidarX_roll**, **xyz_lidarX_roll** e **velodyneX_roll** 

Rotação roll (ao redor do eixo X) do lidar em relação ao robô (geralmente sensorboard, depende de **sensor_reference**).

### **lidarX_pitch**, **xyz_lidarX_pitch** e **velodyneX_pitch** 

Rotação pitch (ao redor do eixo Y) do lidar em relação ao robô (geralmente sensorboard, depende de **sensor_reference**).
**lidarX_shot_size**
### **lidarX_yaw**, **xyz_lidarX_yaw** e **velodyneX_yaw** 

Rotação yaw (ao redor do eixo Z) do lidar em relação ao robô (geralmente sensorboard, depende de **sensor_reference**).

### **lidarX_ray_order**

Ordem dos raios que estão sendo enviados 

### **lidarX_vertical_angles**

Ângulos vertical dos raios, em graus (0 graus seria no nível do lidar)

### **lidarX_sensor_reference** e **xyz_lidarX_sensor_reference**

informa em que a posicao do lidar esta referenciada. 0 para sensorboard, 1 para front_bullbar, 2 para rear_bullbar

### **velodyneX_gps_enable**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_vertical_resolution**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_mapper**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_fov**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_vertical_correction**

Acho que é igual a **lidarX_vertical_angles**.

### **velodyneX_horizontal_correction**

Uma correnção no ângulo horizontal para cada raio, geralmente só usado para ouster.

### **velodyneX_velodyne_package_rate**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_dist_lsb**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_rotation_resolution**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_rotation_max_units**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_velodyne_num_lasers**

Acho que equivale a **lidarX_shot_size** (The number of lasers per shot).

### **velodyneX_velodyne_num_shots**

O número de shots em cada pacote, por exemplo, se velodyneX_velodyne_num_lasers=32 e velodyneX_velodyne_num_shots=10, em cada pacote teremos 10 shots em que cada um tem 32 pontos.

### **velodyneX_min_sensing_distance**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_max_sensing_distance**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_velodyne_msg_buffer_size**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_velodyne_gps_buffer_size**

Não sei, quando descobrir atualPortas usadas para comunicação com o lidar (geralmente configurável no painel de controle do lidar, colocando o IP do lidar no navegador).
ize essa documentação.

### **velodyneX_velodyne_driver_broadcast_freq_hz**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_velodyne_udp_port**

Portas usadas para comunicação com o lidar (geralmente configurável no painel de controle do lidar, colocando o IP do lidar no navegador).

### **velodyneX_velodyne_gps_udp_port**

Portas usadas para comunicação com o lidar (geralmente configurável no painel de controle do lidar, colocando o IP do lidar no navegador).

### **velodyneX_velodyne_upper_header_bytes**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_velodyne_lower_header_bytes**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_gyro_scale_factor**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_temp_scale_factor**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_temp_base_factor**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_accel_scale_factor**

Não sei, quando descobrir atualize essa documentação.

### **velodyneX_velodyne_min_frequency**

Não sei, quando descobrir atualize essa documentação.