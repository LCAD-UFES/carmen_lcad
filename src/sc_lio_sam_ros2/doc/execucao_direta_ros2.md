# Execução ROS2 direta

Os sete wrappers `run_slam_ros2.sh`, `run_lidar_gps_ros2.sh`,
`run_sensors_ros2.sh`, `run_odometry_ros2.sh`, `run_ros2_node.sh`,
`run_lio_sam_node.sh` e `run_preintegration.sh` foram removidos.
Eles apenas repetiam a preparação do ambiente e encaminhavam comandos.

O único arquivo de preparação mantido na raiz é `setup.bash`, usado com
`source`. Ele limpa prefixos e caminhos de bibliotecas previamente carregados,
carrega a distribuição ROS 2 selecionada (Foxy ou Lyrical) e a instalação local.
Usa a distribuição ROS 2 já carregada ou a única instalação suportada em
`/opt/ros`; `SC_LIO_SAM_ROS_DISTRO` permite selecionar explicitamente.
Não inicia processos. Compile a instalação local com a mesma distribuição;
veja a seção de compilação do [README](../README.md).

As 106 chamadas em 23 arquivos de processo agora seguem o formato:

```bash
source /home/eduardopia/carmen_lcad/src/sc_lio_sam_ros2/setup.bash && exec ros2 launch lio_sam run.launch.py ...
```

Os nomes dos módulos, estados iniciais e argumentos foram preservados.
O teste headless também usa comandos nativos. O script de calibração de ruído
IMU foi mantido, pois implementa uma ferramenta funcional, não uma ponte.
Documentos anteriores em `doc` são registros históricos e podem mencionar
os wrappers removidos. O README descreve a execução atual.

Foram verificados sintaxe shell, argumentos do launch e ausência de referências
ativas aos wrappers removidos nos processos e testes. Os dois playbacks
usaram mapas temporários, trechos de 60 segundos e RViz2 em GPU.

```text
Results: /tmp/proccontrol-iara-eq_s6c4w
CARMEN-LCAD - Carnegie Mellon Robot Navigation Toolkit - LCAD Version 0.8.0

CARMEN-LCAD - Carnegie Mellon Robot Navigation Toolkit - LCAD Version 0.8.0

PASS iara poses: 1007 map: /tmp/proccontrol-iara-eq_s6c4w/map
WARNING: RViz2 SIGSEGV only after shutdown; SLAM and map-save checks passed.
```

```text
Results: /tmp/proccontrol-argos-nontafzs
CARMEN-LCAD - Carnegie Mellon Robot Navigation Toolkit - LCAD Version 0.8.0

CARMEN-LCAD - Carnegie Mellon Robot Navigation Toolkit - LCAD Version 0.8.0

PASS argos poses: 1182 map: /tmp/proccontrol-argos-nontafzs/map
WARNING: RViz2 SIGSEGV only after shutdown; SLAM and map-save checks passed.
```

A falha de encerramento do RViz2, quando registrada, continua limitada ao
SIGSEGV posterior ao SIGINT; operação, mapper e mapas são verificados separadamente.
