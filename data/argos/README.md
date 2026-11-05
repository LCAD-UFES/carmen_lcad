# Argos

| Local | Conteúdo |
|---|---|
| `process/` | Processos do proccontrol |
| `parameters/` | Parâmetros CARMEN/ROS, calibrações e modelos OBJ/MTL e de colisão |
| `geodata/` | Rotas, anotações e missões |
| `geodata/graph/` | Grafos de navegação |
| `geodata/maps/` | Mapas |
| `init_vcan_argos.bat` | Preparação das interfaces CAN virtuais |

Execute os processos a partir de `$CARMEN_HOME/bin`:

```bash
cd "$CARMEN_HOME/bin"
./proccontrol ../data/argos/process/process-argos-playback-sc-3d-slam.ini
```

Os caminhos relativos dos processos e parâmetros consideram esse diretório.
