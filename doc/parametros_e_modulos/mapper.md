# Descrição

O módulo mapper é responsável por gerar o mapa do ambiente ao redor, utilizando os dados obtidos dos sensores, gerando tanto o mapa "online" quanto o "offline" quando necessário.

# Parâmetros

Existem dois grandes grupos de parâmetros do mapper, que são o "*mapper_mapping_mode_off*" e "*mapper_mapping_mode_on*", sendo que o primeiro são conjuntos de parâmetros quando ele NÃO está gerando mapas "offline", enquanto o segundo é para quando ele está criando mapas "offline". 

### **mapper_mapping_mode_on_velodyne_range_max** e **mapper_mapping_mode_off_velodyne_range_max**

Determina o range máximo que ele usará para os dados de lidar para gerar mapa. Por exemplo, se esse valor estiver em 70.0, ele irá ignorar qualquer dado de lidar além de 70m para gerar o mapa.

### **mapper_mapping_mode_on_velodyne_range_max_factor** e **mapper_mapping_mode_off_velodyne_range_max_factor**

Opera em conjunto em com o parâmetro "*mapper_mapping_mode_{on/off}_velodyne_range_max*". A distância máxima dos raios de lidar que identificam regiões atrás do robô (mais precisamente no semicírculo atrás dele) são divididos por esse valor. Por exemplo, se esse valor for 2.0 e o valor do "*mapper_mapping_mode_{on/off}_velodyne_range_max*" for 70.0, o mapper utilizará todos os dados de lidar que estão à frente do robô até 70m (a frente nesse caso é o semircírculo frontal) e todos os dados de lidar a 70/2 = 35m atrás do robô (semicírculo traseiro).