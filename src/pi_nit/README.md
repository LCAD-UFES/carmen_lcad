# pi_nit — Neural Image Tracker no Raspberry Pi

Detecção de pessoas em uma **Raspberry Pi 5 + Hailo-8L (AI HAT+ 13 TOPS)**,
usando a imagem das câmeras do CARMEN. O módulo tira a inferência da GPU do PC
e a coloca em um acelerador dedicado de ~5 W, publicando exatamente a mesma
mensagem que o `neural_image_tracker` publica.

```
        PC do CARMEN                                   Raspberry Pi 5
┌───────────────────────────────┐              ┌────────────────────────┐
│ camera_drivers                │              │                        │
│   │ camera_message            │   ZMQ PUSH   │  pi_nit_server.py      │
│   ▼                           │  ──────────► │        │               │
│ pi_nit_client_driver          │   :5560      │        ▼               │
│   • recorta / corrige         │              │   Hailo-8L (yolov8s)   │
│   • letterbox 640×640         │              │        │               │
│   • JPEG                      │   ZMQ PUSH   │        ▼               │
│   • desfaz o letterbox        │  ◄────────── │  caixas 640×640        │
│   • track_id por IoU          │   :5561      │                        │
│   ▼                           │              └────────────────────────┘
│ neural_detector_message_<N>   │
│ carmen_pi_nit_status_message   │
└───────────────────────────────┘
```

Como a saída é a `neural_detector_message` padrão, **qualquer consumidor dessa
mensagem funciona sem alteração** — o `pi_nit` é um substituto direto de um
detector 2D rodando na GPU do PC.

---

## Leia primeiro: o que existe nesta árvore

O `pi_nit` aqui é **autossuficiente**. Duas consequências práticas:

**1. A `neural_detector_message` mora dentro do próprio módulo.** Nesta árvore não
existe um módulo dono dessa interface, então `neural_detector_messages.h`,
`neural_detector_interface.h` e `neural_detector_interface.c` são compilados pelo
`pi_nit` e exportados como `libneural_detector_interface.a`. Se um dia entrar um
módulo dono dela, basta remover os três arquivos daqui e trocar por `-lneural_detector_interface`
no `Makefile` — o resto do código não muda.

**2. Os consumidores citados ao longo deste documento não fazem parte desta árvore.**
`neural_image_tracker`, `multiple_object_tracker` e `image_path_projector` aparecem
no texto por dois motivos legítimos — explicar de onde vem o formato da mensagem, e
descrever o que um consumidor faz com ela (fusão com LiDAR para virar posição em
metros, correção de distorção da caixa, nomes de classe COCO). **As referências a
arquivos e linhas desses módulos são sobre a árvore de origem, não sobre esta.**
Aqui o `pi_nit` publica a mensagem e ponto; quem for consumi-la precisa ser escrito
ou portado.

O que **está** aqui e funciona sozinho: o cliente C++ completo (recorte, letterbox,
JPEG, ZMQ, desfaz o letterbox, `track_id` por IoU), o servidor Python inteiro para o
Raspberry, o teste de enlace, o publicador de câmera a partir de vídeo, e os scripts
de geração da imagem do cartão.

### O que não veio junto (de propósito)

| | por quê | como obter |
|---|---|---|
| pesos YOLO (`*.pt`, ~147 MB) | binários não entram no repositório | `pi_nit_server/download_model.sh`, ou baixe pelo `ultralytics` |
| vídeo de exemplo (`data/pi_nit/pedestres.avi`) | binário | grave um vídeo qualquer com pessoas; veja [`COMO_TESTAR.md`](COMO_TESTAR.md) |

---

## Estrutura

| Arquivo | O que é |
|---|---|
| `pi_nit_client_driver.cpp` | módulo do CARMEN: assina a câmera, fala com o Pi, publica as detecções |
| `pi_nit_zmq_client.{hpp,cpp}` | sockets ZMQ (PUSH de frames, PULL de resultados) |
| `pi_nit_tracker.{hpp,cpp}` | rastreador por IoU que preenche o `track_id` |
| `pi_nit_protocol.h` | formato binário da rede — **espelho de `pi_nit_server/pi_nit_protocol.py`** |
| `pi_nit_messages.h` / `pi_nit_interface.*` | mensagem IPC de status do enlace |
| `pi_nit_link_test.cpp` | teste do enlace sem IPC e sem câmera |
| `pi_nit_camera_publisher.cpp` | publica `camera_message` a partir de um vídeo ou webcam, para testar sem o carro |
| `neural_detector_{messages,interface}.*` | a mensagem de detecção publicada — embutida aqui, veja acima |
| **`pi_nit_server/`** | **tudo o que sobe para o Raspberry** — veja o [README de lá](pi_nit_server/README.md) |
| **`pi_nit_server/image/`** | gera a **imagem pronta do Raspberry** — [README_IMAGEM.md](pi_nit_server/image/README_IMAGEM.md) |
| **[`COMO_TESTAR.md`](COMO_TESTAR.md)** | **roteiro de teste com vídeo em loop** — comece por aqui |
| **[`HARDWARE.md`](HARDWARE.md)** | LiDAR, câmeras e GPU que a percepção exige, e o que dá para mover para outro Raspberry |

---

## Compilando (PC do CARMEN)

Depende de `libzmq3-dev` (o resto já é do CARMEN):

```bash
sudo apt install libzmq3-dev
cd $CARMEN_HOME/src/pi_nit
make
```

Gera `pi_nit_client_driver`, `pi_nit_link_test` e `pi_nit_camera_publisher` em
`$CARMEN_HOME/bin`.

---

## Rodando

Antes: o Raspberry precisa estar instalado e com o serviço no ar — siga
[`pi_nit_server/README.md`](pi_nit_server/README.md).

```bash
./pi_nit_client_driver <camera_model> <msg_number> [<camera_model> <msg_number> ...] [opções]

# uma câmera
./pi_nit_client_driver intelbras1 3 -pi_host 192.168.1.20 -fps 15 -show on

# as 3 câmeras do plano, em um único processo
./pi_nit_client_driver intelbras1 3 intelbras2 4 intelbras3 5 \
    -pi_host 192.168.1.20 -fps 15
```

Os pares seguem a mesma convenção do `neural_image_tracker`: nome do modelo
da câmera (usado para achar a calibração no `carmen.ini`) e número da
mensagem de câmera. Até 5 câmeras (`MAX_CAMERAS`); o `-fps` é **por câmera**,
então 3 câmeras a 15 fps = 45 inferências/s no Raspberry.

Cada câmera tem seu próprio rastreador, sua própria contagem de fps e sua
própria `neural_detector_message`. Uma câmera travada não afeta as outras.

| Opção | Padrão | Descrição |
|---|---|---|
| `-pi_host <ip>` | `192.168.1.20` | endereço do Raspberry |
| `-frame_port <n>` | `5560` | porta de envio de frames |
| `-result_port <n>` | `5561` | porta de retorno das detecções |
| `-fps <n>` | `15` | taxa máxima de envio (`0` = toda imagem que chegar) |
| `-jpeg_quality <n>` | `80` | `1..100`; `0` envia BGR cru (~150 Mbit/s) |
| `-confidence <f>` | `0.4` | confiança mínima publicada |
| `-image <n>` | `0` | índice da imagem dentro da `camera_message` |
| `-undistort <0\|1>` | `1` | corrige a distorção antes de enviar |
| `-person_only <0\|1>` | `0` | descarta tudo que não for pessoa (quem escolhe as classes é o servidor) |
| `-track <0\|1>` | `1` | preenche o `track_id` por IoU |
| `-coco_ids <0\|1>` | `0` | `1` publica o id COCO cru em vez da convenção do MOT (veja abaixo) |
| `-publish <0\|1>` | `1` | `0` detecta e mostra, mas **não** publica — para rodar ao lado do `neural_detector` |
| `-ignore_bottom <f>` | `0` | apaga a base da imagem antes de enviar (o capô); `<1` é fração, `≥1` é pixel |
| `-show <on\|off>` | `off` | janela com as caixas desenhadas |

Também dá para fixar três deles no arquivo de parâmetros do carro, na seção
da câmera:

```ini
intelbras1_pi_nit_host            192.168.1.20
intelbras1_pi_nit_fps             15
intelbras1_pi_nit_ignore_bottom   0.13
```

### O capô do carro vira detecção

A câmera da frente enxerga o capô, e o detector o reconhece como **carro** em
quase todo frame: uma caixa parada, colada na câmera, que o MOT ainda tenta
fundir com o LiDAR. Medido no log `iara_20260730-3`: aparece em 15 de 85 frames
amostrados, sempre no mesmo lugar (`x≈140`, topo da caixa em `y≈426` numa
imagem de 480 linhas).

`-ignore_bottom` apaga essa faixa **antes de enviar**, com o mesmo cinza do
letterbox — a rede nem gasta tempo com ela, e a janela do `-show` mostra
exatamente o que foi ignorado. É melhor que filtrar a detecção depois: não
adianta descartar uma caixa que a rede já pagou para produzir.

```bash
./pi_nit_client_driver intelbras1 1 -pi_host 127.0.0.1 -ignore_bottom 0.13
```

`0.13` são 62 px das 480 linhas, começando na 418 — cobre o capô com folga e
come pouquíssima pista (aquele pedaço fica a 2–3 m do carro, coberto pelo
LiDAR). O parâmetro é **por câmera**, porque só a da frente vê o capô: no
`carmen.ini`, `<modelo>_pi_nit_ignore_bottom`. A linha de comando vale para
todas e tem prioridade.

Para achar o valor de uma câmera nova, rode com `-show on` e suba o número até
o capô sumir; ou meça a caixa que o detector cria em cima dele.

### No process control

```
pi_nit  support  1  0  ./pi_nit_client_driver intelbras1 3 intelbras2 4 intelbras3 5 -pi_host 192.168.1.20
```

---

## Testando tudo no PC, sem o Raspberry

Dá para exercitar a cadeia inteira — letterbox, ZMQ, timestamps, mapeamento
de coordenadas, tracking e publicação IPC — rodando o servidor no próprio PC
com o backend `cpu`, que usa o YOLO do `multiple_object_tracker`. As classes
são as mesmas do COCO, então o que o CARMEN recebe é **indistinguível** do que
vai receber do Hailo.

```bash
VENV=$CARMEN_HOME/data/pi_nit/venv/bin/python3
$VENV -m pip install pyzmq          # uma vez só

# servidor "faz de conta que sou o Raspberry", na própria máquina
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
$VENV pi_nit_server.py --backend cpu --device 0 \
    --weights $CARMEN_HOME/data/pi_nit/weights/yolov8n.pt \
    --batch-size 1 --classes 0,2,3,5,7 --bind 127.0.0.1 --viewer-port 5562

# em outro terminal: o CARMEN normal, apontando para o localhost
./pi_nit_client_driver intelbras1 3 -pi_host 127.0.0.1 -show on
```

O `--batch-size` tem que casar com o número de câmeras, e `--classes` decide o
que volta (o cliente publica o que vier). Medido nesta máquina (RTX 3050
Laptop, câmera do log a 15,1 Hz):

| | |
|---|---|
| Inferência `yolov8n` na GPU | ~11 ms por imagem |
| Taxa entregue ao detector | 13,1 – 14,7 fps |

Não é o desempenho do Hailo — é um substituto para validar a lógica. Se a taxa
entregue ficar bem abaixo da taxa da câmera, **não é descarte "por projeto"**:
veja as duas armadilhas de taxa em [Desempenho](#duas-armadilhas-de-taxa-as-duas-já-corrigidas).

O simulador roda `yolov8n` enquanto o Raspberry roda `yolov8s` — a qualidade
que você vê aqui é um pouco pior que a do Hailo, não melhor.

Sem nenhum modelo, o `--dummy` responde com uma caixa fixa e serve para medir
só a latência da rede:

```bash
python3 pi_nit_server.py --dummy --bind 127.0.0.1
$CARMEN_HOME/bin/pi_nit_link_test 127.0.0.1 -frames 100 -fps 15
```

---

## Mensagens publicadas

**`neural_detector_message_<N>_name`** — a mesma do `neural_image_tracker`
(`neural_detector_messages.h`). As caixas vêm em **coordenadas da imagem
original da câmera**: o letterbox 640×640 é desfeito no PC antes de publicar,
então a fusão com LiDAR e a projeção na imagem continuam válidas.

### De onde vem a coordenada de cada pessoa em metros

**O pi_nit não calcula posição em metros — e os detectores existentes também
não.** Os três publicam a mesma coisa: caixa 2D em pixels da imagem.

| Detector | Compilado por | Como detecta | Onde publica |
|---|---|---|---|
| `neural_detector` | `multiple_object_tracker/` | Python + PyTorch (GPU) | `neural_detector_main.cpp:568` |
| `neural_image_tracker` | `neural_image_tracker/` | DeepStream + TensorRT (GPU) | `neural_image_tracker.cpp:101` |
| `pi_nit_client_driver` | `pi_nit/` | Raspberry + Hailo | `pi_nit_client_driver.cpp` |

Todos chamam `neural_detector_publish_message()`, e a `neural_detector_message`
só tem `x, y, w, h, prob, obj_id, track_id` (`neural_detector_messages.h:11`).
Nenhum metro.

> Cuidado ao ler o código: existe um `neural_detector_main.cpp` **também** na
> pasta `neural_image_tracker/`, mas ele é **código morto** — a regra que o
> compilaria está comentada (`neural_image_tracker/Makefile:152`). O binário
> `neural_detector` sai só de `multiple_object_tracker/`.

Quem transforma caixa 2D em metros é o **`multiple_object_tracker`**: ele
assina a `neural_detector_message` (`multiple_object_tracker_main.cpp:1323`),
funde com o LiDAR (`lidar_to_camera.cpp`) e publica a posição 3D.

```
camera_drivers → neural_detector (2D) → multiple_object_tracker (+LiDAR) → 3D
                        ↑
               pi_nit entra AQUI, no lugar dele
```

Foi exatamente por isso que este módulo publica a `neural_detector_message` em
vez de inventar uma mensagem própria: ele é um **substituto direto do
`neural_detector`**, e tudo a jusante continua funcionando sem alteração.

Para ter os metros, rode o MOT em cima do pi_nit — o mesmo comando de sempre:

```bash
# detecção no Raspberry
./pi_nit_client_driver intelbras1 3 -pi_host 192.168.1.20 -fps 15

# fusão com LiDAR, que produz os metros
./multiple_object_tracker intelbras1 3 -lidar 16 -detect_always 1

# ver a coordenada saindo
$CARMEN_HOME/bin/print_ipc_message carmen_moving_objects_point_clouds_message_0_name
```

> `-detect_always` aparece no README do MOT mas **não existe** como parâmetro
> dele (`multiple_object_tracker_main.cpp:1226`). É ignorado; não faz mal
> passar, só não faz nada.

---

## Três coisas que o MOT exige e ninguém documenta

Ser "a mesma mensagem" não basta. O MOT tem três exigências que não geram
nenhum erro quando não são atendidas — a detecção simplesmente some.

### 1. O `obj_id` é o id COCO **menos 1**

`multiple_object_tracker.cpp:1012` nomeia assim:

| `obj_id` | nome no MOT | classe COCO |
|---|---|---|
| `-1` | `pedestrian` (1×1 m) | 0 pessoa |
| `1` | `Car` | 2 carro |
| `2` | `motorcycle` | 3 moto |
| `4` | `bus` | 5 ônibus |
| `6` | `truck` | 7 caminhão |
| resto | `unknown`, com dimensões de veículo | — |

É a mesma conta do `neural_detector` (`neural_detector_main.cpp:135`:
`p.obj_id = int(preds[i+1]) - 1`). Publicar o id COCO cru joga tudo no `else`
— e pior, `obj_id != -1` liga a **estimativa de orientação de veículo** em
cima de um pedestre (`multiple_object_tracker.cpp:518`).

O `pi_nit_client_driver` subtrai 1 automaticamente. `-coco_ids 1` desliga isso,
para quem precisar dos ids originais em outro consumidor.

### 2. O campo `host` decide **como a caixa é lida**

`multiple_object_tracker_main.cpp:235` e
`camera_viewer_draw_functions.cpp:523` fazem a mesma coisa:

```c
if      (strstr(host, "neural_image_tracker")) { ... rectify_bbox() ... }
else if (strstr(host, "neural_detector"))      { p.x = ...; p.y = ...; }
// não existe else
p.prob = ...; predictions.push_back(p);
```

`carmen_get_host()` devolve `"<nome_do_módulo>@<MÁQUINA>"`
(`global/ipc_wrapper.c:501`). Com `pi_nit_client_driver@...` **nenhum dos dois
ramos executa**: o MOT empurra um `bbox_t` com `x/y/w/h` **não inicializados**
para o vetor de predições, e o `camera_viewer` não desenha nada. Sem erro, sem
aviso.

Por isso o módulo monta o próprio host em `build_detector_host()`:

```
neural_detector_pi_nit@MÁQUINA
```

Casa no `strstr` (nossa convenção de caixa é a do `neural_detector`: canto
superior esquerdo, em pixels da imagem original) e continua identificável no
`print_ipc_message`.

> A correção de verdade seria um `else` nesses dois arquivos — hoje qualquer
> detector novo é ignorado em silêncio. É código compartilhado, então fica
> como decisão de quem cuida do MOT.

### 3. Sem `globalpos`, a detecção é descartada

`multiple_object_tracker_main.cpp:557`:

```c
if (is_full && (globalpos_msg != NULL))
    track_objects(...);
```

Precisa do `localize_ackerman` publicando **e** de todos os LiDARs vivos já
inseridos em `global_points`. Num playback parado, nada disso circula — e a
ausência de caixa no MOT não tem relação com o detector.

---

## Só um publicador por vez

`neural_detector`, `neural_image_tracker` e `pi_nit` publicam na **mesma**
`neural_detector_message_<N>_name`. Dois ao mesmo tempo fazem o MOT receber
duas sequências independentes de `track_id`, e cada objeto troca de identidade
a cada mensagem — parece perda de pacote, mas é briga de fonte. O campo `host`
alternando no `print_ipc_message` é o sintoma.

Para **comparar** duas redes na mesma câmera, rode uma publicando e a outra com
`-publish 0`:

```bash
# a rede python alimenta o MOT
./neural_detector intelbras1 1
./camera_viewer intelbras1 1

# o pi_nit só mostra, na janela dele
./pi_nit_client_driver intelbras1 1 -pi_host 127.0.0.1 -show on -publish 0
```

No `process-playback_iara.ini` isso já está montado como "Receita A" e
"Receita B" — veja [COMO_TESTAR.md](COMO_TESTAR.md).

> **Cuidado com o índice da mensagem de saída.** A entrada é indexada pelo
> *número da mensagem de câmera* (`neural_detector_message_3_name`), mas a
> saída do MOT é indexada pela *posição do argumento na linha de comando*
> (`multiple_object_tracker_main.cpp:466` chama
> `publish_message_generic(cam_index, ...)`). Com uma câmera só, mesmo sendo a
> câmera 3, a saída é sempre `..._message_0_name`.

Os campos úteis de cada `t_point_cloud_struct`
(`moving_objects_messages.h:104`):

| Campo | O que é |
|---|---|
| `object_pose` | posição da pessoa (`x`, `y`, `z`) |
| `linear_velocity`, `lateral_velocity` | velocidade e o desvio padrão de cada uma |
| `orientation` | orientação |
| `length`, `width`, `height` | dimensões estimadas |
| `num_associated` | id do objeto móvel |

**`carmen_pi_nit_status_message`** — saúde do enlace, uma vez por segundo:

```c
int    camera_id, connected, frames_sent, frames_received, detections;
double fps_sent, fps_received, round_trip_ms, inference_ms;
```

`frames_sent - frames_received` é o número de frames descartados de
propósito, quando o Pi não acompanha. Isso é normal e desejado — veja abaixo.

---

## Decisões de projeto

**Descartar frame é melhor que atrasar.** Os dois lados usam `ZMQ_CONFLATE`
com fila de tamanho 1: se a inferência atrasar, o frame velho é jogado fora e
o próximo a ser processado é sempre o mais recente. Em percepção para
navegação, uma detecção de 300 ms atrás é pior que detecção nenhuma.

**Letterbox no PC, não no Pi.** O Raspberry recebe sempre 640×640 e não
precisa saber nada sobre a câmera. O PC guarda a escala e o deslocamento de
cada frame e desfaz o mapeamento quando o resultado volta.

**JPEG por padrão.** 640×640×3 cru são 1,2 MB por frame — 150 Mbit/s a
15 fps. Em JPEG 80 caem para ~70 kB (~10 Mbit/s), e a decodificação custa
2–3 ms no Pi 5. Use `-jpeg_quality 0` só em gigabit dedicado.

**`track_id` no PC.** O Hailo devolve caixas por frame, sem identidade. A
associação por IoU é feita aqui, onde há CPU sobrando, e não no Raspberry.

**Batch de 3, um por câmera.** O servidor junta os frames que chegaram numa
janela de 20 ms — no máximo um por câmera, sempre o mais novo — e manda as 3
imagens ao Hailo em uma única inferência. Medido com `yolov8n` na GPU do PC:
**94 ms por imagem sozinha → 10 ms em batch de 3.**

**Nada de `ZMQ_CONFLATE`.** É tentador (ele guarda só a mensagem mais nova),
mas o CONFLATE não sabe que as mensagens vêm de câmeras diferentes: com 3
câmeras num socket só, ele comeria 2 de cada 3 frames. Em vez disso a fila é
limitada ao número de câmeras e o envio é `DONTWAIT` — quem descarta o frame
velho *de cada câmera* é o servidor, que sabe ler o `camera_id`.

**Um cliente por Raspberry.** Os sockets são PUSH/PULL: se dois
`pi_nit_client_driver` apontarem para o mesmo Pi, as respostas se dividem
entre eles. O `client_id` no protocolo detecta isso e o cliente avisa no
`stderr` — mas a configuração continua errada. Para duas câmeras, use dois
Raspberry (ou duas instâncias do serviço em portas diferentes).

---

## Diagnóstico

Quando não aparece detecção nenhuma, teste **em ordem**:

```bash
# 1. o Raspberry responde? (sem IPC, sem câmera)
$CARMEN_HOME/bin/pi_nit_link_test 192.168.1.20 -frames 30

# 2. com uma imagem de verdade
$CARMEN_HOME/bin/pi_nit_link_test 192.168.1.20 -image /tmp/pessoas.jpg -show

# 3. o módulo está recebendo imagem da câmera?
./pi_nit_client_driver intelbras1 3 -pi_host 192.168.1.20 -show on

# 4. a mensagem está saindo?
$CARMEN_HOME/bin/print_ipc_message neural_detector_message_3_name
```

Se o passo 1 falhar, o problema está no Raspberry ou na rede — vá para a
seção *Problemas comuns* do [README do servidor](pi_nit_server/README.md).

---

## Desempenho esperado

| | Hailo-8L (yolov8s, 640×640) |
|---|---|
| Inferência | 15–20 ms/frame |
| Decodificação JPEG no Pi | 2–3 ms |
| Ida e volta na rede (gigabit) | 5–15 ms |
| **Round-trip total** | **30–50 ms** |
| Taxa sustentada | 15 fps com folga (o 8L fecha ~30 fps no yolov8s) |
| Consumo do conjunto | ~10 W |

Com `yolov8n` a inferência cai para ~8 ms, com perda pequena de precisão em
pessoas distantes.

### Duas armadilhas de taxa, as duas já corrigidas

Medido no simulador (RTX 3050, `yolov8n`, câmera do log a 15,1 Hz):

| | fps entregues ao detector |
|---|---|
| antes | 9 |
| depois | 13,1 – 14,7 |

**A janela do batch.** O `collect_batch` esperava `batch_window_ms` até juntar
`batch_size` frames. Com **menos câmeras que o `batch_size`** o lote nunca
enchia, então **todo frame pagava a janela inteira** — 66 ms viravam 86 ms.
Agora o alvo é o número de câmeras que publicaram nos últimos 2 s.

**O limitador de `-fps`.** O período era medido a partir do **envio**, o que
somava o preparo da imagem (undistort + letterbox + JPEG) ao intervalo. Com a
câmera na mesma taxa do alvo, o frame seguinte chegava sempre alguns ms antes
do prazo e era descartado — um sim, outro não. Agora a comparação é contra uma
grade de tempo, com 10 % de folga para o jitter.

**No Raspberry, `PI_NIT_BATCH_SIZE` tem que ser igual ao número de câmeras.**
Com batch 3 e uma câmera só, o `hailo_person_detector.py:145` completa o lote
**repetindo a última imagem** e descarta os resultados extras: dois terços do
acelerador trabalhando à toa. O serviço avisa isso no log.

### O que melhora a detecção (medido, não achismo)

25 frames do vídeo de teste, letterbox 640, `conf 0.25`:

| | detecções | conf média | ≥ 0,40 |
|---|---|---|---|
| `yolov8n` + jpeg 80 | 179 | 0,712 | 162 |
| `yolov8n` sem jpeg | 168 | 0,733 | 158 |
| `yolov8x` + jpeg 80 | 166 | 0,795 | 160 |
| `yolov8x` sem jpeg | 165 | 0,814 | 162 |

- **Modelo maior quase não acha mais gente** — do `n` para o `x` (o maior que
  existe) o número de detecções não muda; o que sobe é a confiança média.
- **A qualidade do JPEG é irrelevante** — 80 → 95 → sem compressão move a
  confiança em 0,02. Não vale a banda no wifi do Pi.
- **Baixar o corte de confiança rende** — 179 detecções a 0,25 contra 162 a
  0,40, e são justamente as pessoas distantes. O ND usa 0,25.
- **Resolução é a alavanca real** — recortar a região de interesse e mandar só
  ela nos 640 rendeu +15 % de detecções na mesma região, num vídeo que quase
  não encolhe (768×576). Com a câmera em 1280×720 o ganho seria maior.
  **Implementado** em `-roi_top`/`-roi_bottom` (veja abaixo) — ainda falta
  medir o ganho real com uma câmera 1280×720 de verdade.

### Recorte de ROI antes do letterbox — `-roi_top` / `-roi_bottom`

Corta o topo e/ou a base da imagem **antes** do letterbox (ao contrário do
`-ignore_bottom`, que só pinta de cinza mantendo os 640×640 cheios de área
sem informação — este de fato reduz a imagem enviada, sobrando mais pixel de
verdade para a região que interessa depois do resize). Mesma convenção do
`-ignore_bottom`: `< 1` é fração da altura, `≥ 1` é pixel, `0` desliga
(padrão — quem não configurar não muda de comportamento).

```bash
./pi_nit_client_driver intelbras1 3 -pi_host 192.168.1.20 -roi_top 0.15 -roi_bottom 0.10
```

Por câmera, no arquivo de parâmetros do carro:

```ini
intelbras1_pi_nit_roi_top      0.15
intelbras1_pi_nit_roi_bottom   0.10
```

O ganho é maior quanto mais larga for a câmera: numa 640×480 (quase quadrada)
o letterbox já desperdiça pouco; numa 1280×720 (aspecto 16:9) o letterbox
pinta ~44 % da imagem de cinza (140 px em cima e embaixo, numa entrada de
640×640) — cortar essa faixa antes do resize usa esses pixels para imagem de
verdade em vez de padding.

> Como o `-ignore_bottom`, o corte é por câmera porque depende da montagem —
> a fração certa se acha do mesmo jeito: `-show on` e sobe o valor até a
> faixa desnecessária (céu no topo, capô/pista muito perto embaixo) sumir sem
> cortar gente. As coordenadas da `neural_detector_message` continuam saindo
> na imagem original inteira — o desfazer do recorte é automático, não muda
> nada a jusante (MOT, `camera_viewer`, etc.).

#### Testando o ROI sem o CARMEN — `--roi-top` / `--roi-bottom`

O `tools/test_client.py` tem as mesmas duas opções (mesma convenção, mesmo
recorte antes do letterbox e o mesmo desfazer no retorno), para conferir o
efeito do ROI com detecção de verdade sem precisar compilar o lado C++:

```bash
python3 tools/test_client.py --host 127.0.0.1 \
    --video $CARMEN_HOME/data/pi_nit/pedestres.avi \
    --roi-top 0.15 --roi-bottom 0.10 --show
```

Validação feita assim, no `pedestres.avi` (768×576), 9 frames comparados
com e sem ROI contra o mesmo servidor `yolov8n`:

| | detecções de pessoa | conf. média |
|---|---|---|
| sem ROI | 27 | 0,811 |
| com ROI `0.15`/`0.10` | 29 | 0,793 |

As **mesmas** pessoas voltam no mesmo lugar da imagem original: o
deslocamento máximo em Y entre as duas execuções foi de **2 px** — é a
confirmação de que o `crop_y` é somado de volta corretamente. O carro e o
caminhão do alto da imagem somem com o ROI ligado, o que é o efeito
desejado: eles ficam na faixa cortada (linhas 44–100, contra o corte em 86).

Como o README já previa, num vídeo quase 4:3 o ganho é pequeno — a alavanca
real é uma câmera 16:9, onde o letterbox desperdiça muito mais.

---

## Qual versão do YOLO usar

Duas perguntas diferentes, que pedem duas fontes diferentes:

1. **Qual rede detecta melhor?** — isso viaja bem entre hardwares (o
   *ranking* entre `n`/`s`/`m`/famílias é praticamente o mesmo na GPU do PC e
   no Hailo; só a escala de tempo muda). Dá para responder testando no PC.
2. **Qual rede fecha os 45 fps (3 câmeras × 15 fps) no Hailo-8L?** — isso
   **não** dá para medir no PC. É preciso o número oficial do chip.

### 1. Qualidade — `tools/compara_versoes_yolo.py`

Roda vários pesos nos **mesmos** frames do vídeo de teste e mede detecções,
confiança média e quantas passam de 0,40 — a mesma metodologia da tabela
acima, agora automatizada para qualquer lista de pesos:

```bash
python3 tools/compara_versoes_yolo.py
python3 tools/compara_versoes_yolo.py --weights yolov8n.pt,yolov8s.pt,yolo11n.pt,yolo11s.pt
```

Medido nesta máquina (CPU, 1 núcleo — por isso o tempo aqui **não** é o do
Hailo, só serve para comparar os pesos entre si), 40 frames, `conf 0,25`:

| peso | parâmetros (M) | ms/img (CPU deste teste) | detecções | conf. média | ≥ 0,40 |
|---|---|---|---|---|---|
| `yolo11n` | 2,6 | 84,8 | 236 | 0,742 | 231 |
| `yolov10n` | 2,3 | 91,7 | 243 | 0,677 | 220 |
| `yolov8n` | 3,2 | 98,8 | 240 | 0,756 | 231 |
| `yolov5su` | 9,1 | 203,0 | 238 | 0,770 | 232 |
| `yolov10s` | 7,2 | 230,9 | 239 | 0,764 | 227 |
| `yolo11s` | 9,4 | 210,5 | 241 | **0,780** | 233 |
| `yolov8s` | 11,2 | 237,6 | 236 | 0,779 | 230 |
| `yolov8m` | 25,9 | 546,1 | 236 | **0,814** | 233 |

Confirma para a família toda o que já estava medido só para `n` vs. `x`:
**o número de detecções mal se move** (236–243 em todos os 8 pesos) — o que
sobe com o tamanho do modelo é a confiança. `yolov10n` é a exceção: mais
detecções brutas, mas a confiança média mais baixa da tabela (mais caixa
"duvidosa").

### 2. Throughput no Hailo-8L — tabela oficial do Model Zoo

O [`HAILO8L_object_detection.rst`](https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8L/HAILO8L_object_detection.rst)
dá `mAP` e `fps` **medidos no chip**, em batch 1 e batch 8 (não existe
número oficial de batch 3, mas 1 já é o pior caso — sem o ganho do batching
que o `pi_nit_server` faz):

| peso | mAP (hw) | fps batch 1 | fps batch 8 | folga sobre 45 fps (bs=1) |
|---|---|---|---|---|
| `yolov6n` | 32,5 | 356 | 356 | 7,9× |
| `yolov8n` | 36,4 | 202 | 438 | 4,5× |
| `yolov11n` | 37,8 | 157 | 371 | 3,5× |
| `yolov10n` | 36,6 | 150 | 359 | 3,3× |
| `yolov5s` | 34,1 | 124 | 243 | 2,8× |
| `yolov8s` ⭐ atual | 44,0 | 110 | 208 | 2,4× |
| `yolov11s` | 45,5 | 91,9 | 192 | 2,0× |
| `yolov10s` | 45,0 | 87,6 | 187 | 1,9× |
| `yolov8m` | 49,2 | 50,8 | 87,0 | **1,13×** |
| `yolov8l` | 51,8 | 26,1 | 40,7 | abaixo do alvo |

`yolov8m` já está na margem — qualquer variação (rede Wi-Fi, jitter do
batch) pode furar os 15 fps por câmera. `yolov8l` não fecha nem em batch 8.

### 3. Medido no Hailo-8L real (11/08/2026)

As duas tabelas acima são estimativa. Esta é a medida — `hailortcli run
<hef>`, batch 1, sem rede nem ZMQ, direto no Hailo-8L do `pi-nit`
(firmware **4.20.0** — o README antigo dizia 4.24.0; a placa real é mais
antiga que isso, confira sempre com `hailortcli fw-control identify` antes
de comparar números). Processo documentado em
[`tools/valida_novo_modelo.sh`](pi_nit_server/tools/valida_novo_modelo.sh):
para a placa em produção, é preciso parar o `pi_nit_server` durante a
medição — **o Hailo-8L só aceita um processo por vez**, então não dá para
comparar dois modelos ao vivo no mesmo hardware sem essa parada.

| peso | fps medido (bs=1) | fps oficial (bs=1) | % do oficial | mAP (hw) |
|---|---|---|---|---|
| `yolov8s` ⭐ produção | **58,13** | 110 | 53% | 44,0 |
| `yolov11s` | 39,58 | 91,9 | 68% | 45,5 |
| `yolov8x` | 8,99 | 16,2 | 55% | 52,9 |

O padrão se repete nos três: **a placa real entrega bem menos que a tabela
do fabricante** (53-68%, não é defeito, é conhecido na comunidade do
Hailo). O que muda a decisão é que a *proporção entre os modelos* também
piorou para o `yolov11s` — oficialmente ele rendia 84% da velocidade do
`yolov8s` (91,9/110), aqui rendeu só 68% (39,58/58,13), e já fica **abaixo
dos 45 fps mesmo em batch 1**, antes de qualquer overhead de rede/JPEG.
`yolov8x` nem chegou perto (8,99 fps) e está descartado sem dúvida.

### Decisão

**Mantido `yolov8s`.** O ganho de mAP do `yolov11s` (44,0 → 45,5) não
compensa a margem de fps perdida no hardware real — e o `yolov8x`, apesar
do mAP mais alto de todos (52,9), está fora de cogitação para o alvo de 3
câmeras. Reavaliar só se o alvo de fps mudar (menos câmeras, ou tolerância
a fps mais baixo por câmera).

### Juntando as três tabelas

- **`yolov8s` (o padrão do `download_model.sh`) é a escolha confirmada**,
  agora com medida real, não só estimativa: folga de 58,13/45 ≈ 1,3× em
  batch 1, e o batch de 3 da pipeline ainda soma folga em cima disso.
- **`yolov11s` foi descartado** depois de medido — parecia a candidata mais
  forte na tabela oficial (mAP mais alto, boa folga teórica), mas a placa
  real mostrou uma proporção pior que o esperado e ficou abaixo da meta já
  em batch 1.
- **`yolov10n`/`yolov11n`/`yolov6n` continuam candidatas não testadas** se
  a prioridade virar fps (mais câmeras, por exemplo) — ainda não medidas no
  hardware real, só na tabela oficial.
- **`yolov8m`/`yolov8l` seguem sem sentido testar** com o alvo atual de 3
  câmeras — a folga teórica já não compensava o ganho de mAP, e agora
  sabemos que a placa real entrega menos ainda que a tabela promete.

Para trocar de modelo (e para qualquer candidata nova, meça no hardware
real antes de decidir — a tabela do fabricante sozinha não é suficiente,
como este teste mostrou):

```bash
sudo ./download_model.sh yolov11s      # baixa o .hef compilado do Model Zoo
# edite /etc/pi_nit/pi_nit_server.conf: PI_NIT_HEF=/opt/pi_nit/models/yolov11s_h8l.hef
sudo systemctl restart pi_nit_server
```

> O `download_model.sh` aceita qualquer nome do Model Zoo (`yolov11s`,
> `yolov10n`, `yolov6n`, ...) — só confirme que existe HEF publicado para
> `hailo8l` (nem todo modelo do Model Zoo tem; os da tabela acima têm).
> Para medir de verdade sem arriscar a produção, use
> [`tools/valida_novo_modelo.sh`](pi_nit_server/tools/valida_novo_modelo.sh)
> ou, manualmente: `sudo systemctl stop pi_nit_server`, `hailortcli run
> <hef>`, `sudo systemctl start pi_nit_server` — nessa ordem, sempre
> conferindo `systemctl is-active pi_nit_server` antes e depois. Depois de
> trocar de verdade, repita o [Teste 2 do
> `COMO_TESTAR.md`](COMO_TESTAR.md#teste-2--no-raspberry-de-verdade) com a
> pipeline completa (ZMQ, batch de 3, JPEG), não só o `hailortcli run`
> isolado.
