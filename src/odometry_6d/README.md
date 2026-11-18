# odometry_6d — odometria 6D por velocidade

Módulo novo do carmen para odometria em **6 graus de liberdade descrita só por velocidades**:
linear, angular e aceleração angular, no referencial do corpo do robô. Não existe pose na
mensagem. Quem consome integra no próprio relógio.

A primeira fonte é o **Unitree Go2 (ARGOS)**, e o primeiro consumidor é a **predição 4 do
`localize_ackerman`** (filtro de partículas). A ideia é que esta vire uma nova forma de
odometria no carmen, a ser instruída aos poucos nos outros módulos.

```
Go2 (DDS, rt/utlidar/robot_odom)
   └─> odometry_6d_go2 ──carmen_odometry_6d──> localize_ackerman (prediction_type 4)
              └─ (opcional) carmen_fused_odometry ─> localize -mapping_mode on ─> mapper
```

## A mensagem: `carmen_odometry_6d_message`

Definida em `odometry_6d_messages.h`. A interface (`odometry_6d_interface.h`,
`libodometry_6d_interface.a`) tem `subscribe`, `unsubscribe`, `define` e `publish`.

| Campo | Unidade | Referencial | O que é |
|---|---|---|---|
| `vx, vy, vz` | m/s | corpo | velocidade linear (x para frente, y para a esquerda, z para cima) |
| `wx, wy, wz` | rad/s | corpo | velocidade angular (`wz` = taxa de guinada) |
| `alpha_x, alpha_y, alpha_z` | rad/s² | corpo | aceleração angular |
| `timestamp` | s | relógio do carmen | `carmen_get_time()` da amostra |
| `host` | | | |

```c
#define CARMEN_ODOMETRY_6D_NAME "carmen_odometry_6d"
#define CARMEN_ODOMETRY_6D_FMT  "{double,double,double,double,double,double,double,double,double,double,string}"
```

### Por que não tem pose

A odometria "de pose" (x, y, θ integrados por quem publica) vem num referencial que **deriva**
e que pode **resetar**: o Go2 zera ou salta a dele quando senta, levanta ou troca de modo. O
consumidor acaba usando só a diferença entre duas poses, e precisa adivinhar quando houve um
salto. Com velocidades isso não existe: cada amostra é independente, e quem integra decide a
taxa, o referencial e o que fazer com buracos de tempo.

### Por que a aceleração angular

Com `alpha` dá para integrar a guinada no intervalo com um termo de 2ª ordem
(`Δθ = wz·dt + ½·alpha_z·dt²`), o que importa nas curvas e quando a taxa de mensagens cai. Ela
também é uma entrada natural para os próximos usos (detecção de patinação ou de impacto,
ruído adaptativo no filtro, controle).

## Publicador do Go2: `odometry_6d_go2`

`odometry_6d_go2_main.cpp`.

- Assina o `nav_msgs::msg::dds_::Odometry_` que o próprio Go2 publica por DDS
  (padrão `rt/utlidar/robot_odom`: fusão das pernas, IMU e do lidar interno) e usa **só o
  `twist`** (velocidades). A pose do Go2 é ignorada.
- `alpha` é a derivada da velocidade angular entre amostras, com um passa-baixa
  (`-alpha_filter`, 0..1; 1 = sem filtro). Na primeira amostra, ou depois de um buraco maior que
  0,5 s, `alpha = 0`.
- Só publica amostra nova, uma vez cada. A callback do DDS apenas guarda a amostra; quem publica
  no IPC é um timer do carmen (200 Hz). Publicar IPC da thread do DDS corrompe o heap, pelo mesmo
  motivo do `carmen_get_host()`: veja o `xsens_argos_go2`.

| Opção | Padrão | |
|---|---|---|
| `-network_interface` | (vazio) | interface do cabo do Go2 (no ARGOS, `eth0`) |
| `-domain_id` | 0 | domínio DDS |
| `-odom_topic` | `rt/utlidar/robot_odom` | tópico do `Odometry_` |
| `-alpha_filter` | 0.2 | passa-baixa da aceleração angular |
| `-print` | off | imprime `ODOM6D v ... w ... alpha ...` a cada amostra |
| `-fused_odometry` | off | também publica `carmen_fused_odometry_message` (ver abaixo) |
| `-initial_x/_y/_theta` | 0 | âncora da fused_odometry |

### `-fused_odometry on`: mapear só com odometria

Integra as velocidades a partir de `(-initial_x, -initial_y, -initial_theta)` e publica a pose
como `carmen_fused_odometry_message`. Com `localize_ackerman -mapping_mode on`, isso vira o
globalpos e o `mapper -mapping_mode on` monta o mapa só com a odometria do robô, sem SC-LIO-SAM.
Uma `carmen_localize_ackerman_initialize` ("Pose inicial" no app ou no navigator_gui2)
reancora a pose. Veja `data/argos/process/process-argos-mapear-odom6d.ini`.

## Consumidor: predição 4 do `localize_ackerman`

Liga com `localize_ackerman_prediction_type = 4`, ou com
`-localize_ackerman_prediction_type 4` na linha de comando, sem mexer no `.ini`.

1. **A cada mensagem** (`carmen_localize_ackerman_accumulate_odometry_6d`, assinatura com
   `CARMEN_SUBSCRIBE_ALL`), o localize integra o deslocamento no referencial do corpo na última
   predição, pelo ponto médio do arco:
   ```
   Δθ    = wz·dt + ½·alpha_z·dt²
   θm    = θ_acum + Δθ/2
   x_acum += (cos θm·vx − sin θm·vy)·dt
   y_acum += (sin θm·vx + cos θm·vy)·dt
   θ_acum += Δθ
   ```
2. **A cada scan do lidar** (`carmen_localize_ackerman_incorporate_odometry_6d`), aplica
   `(x_acum, y_acum, θ_acum)` em cada partícula, girado pela orientação dela, com ruído
   proporcional ao que foi andado e girado, e zera o acumulado. A partícula 0 segue a odometria
   pura. Parado, as partículas só recebem o ruído de resolução do mapa e o lidar quase não
   precisa corrigir.
3. Sem odometria 6D por mais de `odom6d_timeout`, cai na predição 3 (odometria + IMU), ou na 1
   se também não houver IMU.

Parâmetros (todos opcionais, com padrão no código; prefixo `localize_ackerman_`):

| Parâmetro | Padrão | |
|---|---|---|
| `odom6d_xy_noise` | 0.05 | desvio para frente, fração do deslocamento |
| `odom6d_lateral_noise` | 0.02 | desvio lateral, fração do deslocamento |
| `odom6d_theta_noise` | 0.05 | desvio de guinada, fração da rotação |
| `odom6d_theta_noise_per_meter` | 0.01 | desvio de guinada por metro andado (rad/m) |
| `odom6d_max_speed` | 5.0 | amostra com \|v\| acima disso (m/s) é descartada |
| `odom6d_timeout` | 0.5 | s sem mensagem para cair na predição 3 |

A posição do robô no mapa continua vindo do filtro de partículas: pose inicial mais correção do
lidar. A odometria só move as partículas entre um scan e outro.

## Compilar

O módulo está no `PACKAGES` do `src/Makefile`, antes do `localize_ackerman`.

- A **interface** compila em qualquer máquina, e é o que o localize precisa.
- O **`odometry_6d_go2`** só compila onde existe o unitree_sdk2 original em `~/unitree_sdk2`
  (ou `make UNITREE_SDK2_HOME=...`). Sem ele, o Makefile simplesmente pula o binário.
- No ARGOS existe uma segunda cópia do SDK em `/usr/local`, cuja `libddsc` foi compilada com
  memória compartilhada: os headers dela não compilam o `Odometry_`, e a lib derruba o SDK em
  tempo de execução. Por isso o binário é ligado com caminho completo e **RPATH**
  (`--disable-new-dtags`) para `~/unitree_sdk2/thirdparty/lib`, que vence o `LD_LIBRARY_PATH` do
  `.bashrc` do ARGOS.

## Testar

```bash
# no ARGOS, com o central e o param_daemon no ar
./odometry_6d_go2 -network_interface eth0 -print on
```
Parado, `v` e `w` devem ficar perto de zero e `alpha` oscilar em torno de zero. Andando para
frente, `vx > 0`; girando para a esquerda, `wz > 0`.

Process prontos em `data/argos/process/`:
- `process-argos-localize-odom6d.ini`: localize (predição 4) sem SC-LIO-SAM, robô autônomo;
- `process-argos-mapear-odom6d.ini`: mapeamento 2D só com a odometria (`-fused_odometry on`).

## A fazer

- Confirmar no Go2 que o `twist` do `rt/utlidar/robot_odom` vem no referencial do **corpo**
  (o padrão do `nav_msgs/Odometry` é o `child_frame_id`). Se vier no referencial de odometria,
  é preciso girar pela guinada antes de publicar.
- Medir o ruído real (parado e em linha reta) e ajustar os `odom6d_*`.
- Outras fontes além do Go2 (odometria de rodas + IMU, odometria visual) publicando a mesma
  mensagem.
- Ensinar os outros consumidores (fused_odometry, mapper, controle) a usar a mensagem.
