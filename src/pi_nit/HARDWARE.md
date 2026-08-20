# Hardware da percepção — o que cada peça exige

Levantamento feito lendo o código do `multiple_object_tracker` (MOT), do
`neural_image_tracker` (NIT), do `neural_detector` (ND) e do `pi_nit`, mais o
arquivo de parâmetros `carmen-sensorbox.ini`. Serve para decidir o que dá
para tirar do PC e o que não dá.

## A cadeia

```
LiDAR (Ethernet) ──────────────────────────────────┐
                                                   ├─► multiple_object_tracker ─► objetos móveis 3D
Câmera IP (RTSP) ─► camera_drivers ─► detector 2D ──┘
                                          ↑
                       neural_detector (PyTorch)      ─┐
                       neural_image_tracker (DeepStream) ├─ os três fazem a MESMA coisa:
                       pi_nit (Raspberry + Hailo)      ─┘   caixa 2D em pixels
```

Os três detectores publicam a mesma `neural_detector_message` e são
intercambiáveis. Quem transforma caixa 2D em metros é sempre o MOT, fundindo
com o LiDAR.

### ND e NIT são programas diferentes

Confusão fácil de fazer, porque os nomes se misturam:

| Binário | Sai da pasta | Motor | Precisa de |
|---|---|---|---|
| `neural_detector` | `multiple_object_tracker/` | CPython embutido + PyTorch/ultralytics | GPU NVIDIA + CUDA |
| `neural_image_tracker` | `neural_image_tracker/` | DeepStream + TensorRT | GPU NVIDIA + CUDA + **DeepStream SDK** |

São **dois programas distintos que fazem o mesmo trabalho**. O NIT é o caminho
otimizado (TensorRT); o ND é o caminho flexível (PyTorch).

> Armadilha: existe um `neural_detector_main.cpp` **também** dentro de
> `neural_image_tracker/`, quase idêntico ao do MOT. Ele **não é compilado** —
> a regra está comentada em `neural_image_tracker/Makefile:152`. Não gaste
> tempo lendo esse arquivo achando que é o que roda.

---

## 1. LiDAR — obrigatório para ter posição em metros

Modelos reconhecidos pelo código:

| Fabricante | Modelos |
|---|---|
| Ouster | `OS032`, `OS132`, `OS164`, `OS2` |
| Velodyne | `HDL32` (HDL-32E), `VLP16` (Puck) |
| RoboSense | `RS16` |

Todos são Ethernet (IP + porta UDP). O que está configurado no
`carmen-sensorbox.ini`:

| LiDAR | Modelo | IP | Porta |
|---|---|---|---|
| lidar0 / lidar1 | OS132BH (Ouster OS-1, 32 feixes) | 192.168.1.200 | 7506 / 7502 |
| lidar2 / lidar3 | OS164 (Ouster OS-1, 64 feixes) | 192.168.0.200 | 7501 |
| lidar4 | RS16 (RoboSense, 16 feixes) | 192.168.1.200 | — |
| lidar5 / lidar6 | OS132BH | 192.168.1.205 | — |
| lidar7 / lidar8 | OS032 / OS132 | 192.168.0.207 / .208 | — |

**Até 16 LiDARs simultâneos** — `MAX_NUMBER_OF_LIDARS_NEURAL 17` em
`multiple_object_tracker_main.cpp:52`: 16 handlers (índices 0–15) mais o
índice 16, reservado ao driver `velodyne` legado. Na prática o `process.ini`
usa 1 ou 2:

```
./multiple_object_tracker intelbras2 2 -lidar 5 -lidar 6
./multiple_object_tracker intelbras1 1 -velodyne
```

Alcance com que o módulo trabalha: `MAX_RANGE 100.0` m e
`MAX_DIST_TO_PEDESTRIAN_TRACK 45.0` m. **Para pedestre, um LiDAR de 16 feixes
já resolve** — os de 64 feixes existem por causa do resto da percepção.

## 2. Câmera — IP, RTSP, até 5

| | Modelo | Endereço | Resolução |
|---|---|---|---|
| intelbras1 | `ip_ffmpeg` | `rtsp://…@192.168.1.116:554` | 640×480 (`subtype=1`) |
| intelbras2 | `vip1020` | `rtsp://…@192.168.1.114:554` | 640×480 |
| intelbras3 | `vip1020` | `rtsp://…@192.168.1.113:554` | 640×480 |

`subtype=0` dá 1280×720. Os 640×480 configurados são exatamente o que o
`pi_nit` espera (viram 640×640 por letterbox, sem distorcer).

## 3. GPU — quem precisa e quem não precisa

| Binário | GPU? | Evidência |
|---|---|---|
| `neural_detector` | **Sim** | Embute Python (`Python.h`, `PyImport`), roda YOLO em PyTorch; o venv tem `torch 2.1.0+cu121` |
| `neural_image_tracker` | **Sim, e mais** | DeepStream (`/opt/nvidia/deepstream/`) + TensorRT + `-lcuda`; o Makefile tem caso especial para CUDA 12.8 |
| `multiple_object_tracker` | **Não** | Nenhum `.cu`, nenhum `-lcuda`/`-lcudart`. Só lê a versão do `nvcc` para gerar macros de compilação |
| `pi_nit_client_driver` | **Não** | Letterbox, JPEG e IoU em CPU; a rede neural roda no Hailo |

**A fusão com LiDAR roda em CPU.** A GPU é da detecção — e é exatamente a
peça que o `pi_nit` substitui.

## 4. Calibração — pré-requisito de montagem

Não é hardware, mas sem isso a fusão não fecha. O MOT exige no arquivo de
parâmetros, por câmera:

- **intrínsecos**: `fx`, `fy`, `cu`, `cv`, `pixel_size`
- **pose 6-DOF da câmera**: `x, y, z, roll, pitch, yaw`
- **pose 6-DOF do LiDAR**: idem

Há o modo `-calibrate_camera 1`, que trabalha junto com o `camera_viewer` —
é para isso que serve a `camera_viewer_message` que o MOT assina.

---

# Colocar mais coisa no Raspberry — o que vale e o que não vale

## Por que o `pi_nit` coube num Raspberry

O serviço no Pi **não tem nenhuma dependência do CARMEN**. É Python puro com
`pyzmq`, `numpy`, `opencv` e `hailo_platform`. Recebe bytes, devolve bytes.
Foi isso que tornou o porte viável.

## Opção A — segundo Raspberry para mais câmeras ✅ recomendado

**Esta é a que vale a pena, e já funciona com o código de hoje.**

Cada Pi + Hailo-8L dá conta de **3 câmeras a 15 fps** (medido: batch de 3,
~10 ms por imagem no substituto de GPU; o Hailo faz ~15–20 ms). Precisa de 6
câmeras? Segundo Pi.

Requisitos: nada além de outro kit igual.

| Item | Quantidade |
|---|---|
| Raspberry Pi 5 (2 GB serve) | 1 |
| Hailo AI HAT+ 13 TOPS (Hailo-8L) | 1 |
| Fonte 27 W USB-C oficial | 1 |
| Dissipador ativo | 1 |
| Porta no switch gigabit | 1 |

Como subir:

```bash
# gera a imagem do segundo Pi com outro IP e outro hostname
cd $CARMEN_HOME/src/pi_nit/pi_nit_server/image
sudo ./build_pi_image.sh --offline --ip 192.168.1.21 --hostname pi-nit-2

# no PC, um processo por Raspberry
./pi_nit_client_driver intelbras1 3 intelbras2 4 intelbras3 5 -pi_host 192.168.1.20
./pi_nit_client_driver intelbras4 6 intelbras5 7 intelbras6 8 -pi_host 192.168.1.21
```

Lembre da regra: **um `pi_nit_client_driver` por Raspberry**. Os sockets são
PUSH/PULL; dois clientes no mesmo Pi dividiriam as respostas entre si. O
`client_id` detecta e avisa no `stderr`, mas a configuração continua errada.

## Opção B — segundo Raspberry rodando o MOT ❌ não compensa

Seria o passo lógico ("tirar também a fusão do PC"), mas o custo é
desproporcional. O MOT linka **mais de 45 bibliotecas**:

```
boost (build próprio em /usr/local/carmen_boost), PCL 1.8, Bullet
(BulletCollision, BulletDynamics, BulletSoftBody, LinearMath), libtf,
Eigen, X11, libkml (kmlbase, kmldom, kmlengine), libsickldmrs2,
prob_models, OpenCV (core, imgproc, calib3d)
+ ~20 bibliotecas internas do CARMEN (velodyne_interface, mapper_interface,
  behavior_selector_interface, localize_ackerman_interface, map_io, …)
```

Colocar isso num Pi significa **portar praticamente a árvore inteira do CARMEN
para ARM64**, incluindo o boost customizado. É outro projeto, não uma tarde
de trabalho.

E o ganho seria pequeno: a fusão roda em CPU e o Pi 5 (4× Cortex-A76 a
2,4 GHz) é bem mais fraco que o PC. Um Ouster OS1-64 a 10 Hz entrega
**~655 mil pontos/s** (64 × 1024 × 10); um VLP16, ~288 mil/s. Projetar tudo
isso na imagem e agrupar é trabalho pesado de ponto flutuante.

**Antes de considerar essa opção, meça** quanto o MOT realmente consome hoje:

```bash
./multiple_object_tracker intelbras1 1 -velodyne &
top -p $(pgrep multiple_object_tracker)
```

Se ele já usa mais de ~50% de um núcleo do PC, não cabe num Pi 5 — e a
decisão está tomada sem precisar portar nada.

## Opção C — Raspberry só com o driver do LiDAR ❌ não faz sentido

Parece atraente ("o Pi lê o LiDAR e manda pronto"), mas **o LiDAR já é
Ethernet**. Ele entrega os pacotes UDP direto no PC, sem intermediário. Pôr
um Raspberry no meio só adiciona um salto, latência e mais uma coisa para
quebrar — a nuvem de pontos teria que atravessar a rede de qualquer jeito,
e ainda maior depois de desempacotada.

Só passaria a fazer sentido se o Pi fizesse a **redução** dos dados antes de
mandar (filtrar, agrupar, mandar só os clusters). Aí já é a Opção D.

## Opção D — serviço de fusão enxuto no Raspberry ⚠️ possível, é reescrita

O caminho honesto para tirar a fusão do PC não é portar o MOT: é escrever um
serviço pequeno no Pi, no mesmo molde do `pi_nit`, que faça **só a conta da
fusão** e fale o mesmo protocolo binário ZMQ.

O que precisaria, no mínimo:

1. Receber a nuvem de pontos (o Pi lê o LiDAR direto por UDP)
2. Projetar os pontos na imagem — precisa das matrizes de calibração,
   hoje em `lidar_to_camera.cpp`
3. Pegar os pontos dentro de cada caixa, agrupar, tirar o centroide
4. Devolver `(x, y, z)` por pessoa

**Requisitos**: Pi 5 de **4 GB ou 8 GB** (a nuvem de pontos ocupa bem mais
memória que uma imagem 640×640; os 2 GB do detector não servem aqui), e as
poses de calibração replicadas no Pi.

**Não é rastreamento nem velocidade** — só a posição instantânea. Velocidade,
orientação e associação temporal continuariam no PC, ou teriam que ser
reescritas também.

Antes de investir nisso, vale responder: **o PC está sobrecarregado com a
fusão?** Se não estiver, a Opção A (mais câmeras) é o único motivo real para
um segundo Raspberry.

---

## Resumo

| Objetivo | Vale a pena? | Custo |
|---|---|---|
| Tirar a GPU da detecção | **Já feito** (`pi_nit`) | 1 Pi + Hailo por 3 câmeras |
| Mais de 3 câmeras | **Sim, Opção A** | outro kit igual, zero código novo |
| Tirar a fusão LiDAR do PC | Não pelo porte do MOT (Opção B) | portar a árvore do CARMEN para ARM64 |
| Tirar a fusão LiDAR do PC | Talvez pela Opção D | reescrever a fusão, Pi de 4–8 GB |
| Pi só para o LiDAR | **Não** (Opção C) | o LiDAR já é Ethernet |

E o que **não** sai do carro de jeito nenhum: o **LiDAR**. É ele que dá a
profundidade — a câmera sozinha não dá metros.
