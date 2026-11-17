# Como testar o pi_nit com um vídeo em loop

Roteiro para ver a detecção de pessoas acontecendo em tempo real, com um
vídeo de pedestres rodando em loop. Serve tanto para testar **só no PC**
(antes do Raspberry existir) quanto para testar **o Raspberry de verdade**.

O vídeo já está baixado em:

```
$CARMEN_HOME/data/pi_nit/pedestres.avi
```

É o `vtest.avi` do OpenCV — 768×576, 10 fps, 80 s de gente andando numa
praça. Se sumir, baixe de novo com:

```bash
mkdir -p $CARMEN_HOME/data/pi_nit
curl -L -o $CARMEN_HOME/data/pi_nit/pedestres.avi \
  https://raw.githubusercontent.com/opencv/opencv/4.x/samples/data/vtest.avi
```

---

## Teste 0 — o caminho inteiro no PC, sem Raspberry e sem rede neural

O teste mais barato de todos, e o primeiro a rodar depois de compilar: sobe o
servidor em modo `--dummy` na própria máquina. Não há inferência — cada frame
volta com uma caixa fixa no centro — mas **todo o resto é exercitado de
verdade**: o protocolo binário, o batch, o letterbox, o mapeamento das
coordenadas de volta para os pixels da imagem original, o tracking e a
publicação IPC.

**Preparo (uma vez só):**

```bash
python3 -m venv --system-site-packages $CARMEN_HOME/data/pi_nit/venv
$CARMEN_HOME/data/pi_nit/venv/bin/pip install pyzmq
```

O `--system-site-packages` é o que faz o venv enxergar o `python3-opencv` e o
`python3-numpy` do sistema. No Ubuntu 26.04 o Python é o **3.14**, e ainda não
há wheel de `opencv-python-headless` para ele — o `requirements.txt` só se
resolve reaproveitando os pacotes do sistema. É a mesma escolha do `install.sh`
do Raspberry, feita lá por outro motivo (o `hailo_platform`).

Baixe também o vídeo de exemplo, se ainda não estiver no disco:

```bash
mkdir -p $CARMEN_HOME/data/pi_nit
curl -L -o $CARMEN_HOME/data/pi_nit/pedestres.avi \
  https://raw.githubusercontent.com/opencv/opencv/4.x/samples/data/vtest.avi
```

### 0a — só o enlace ZMQ, sem CARMEN nenhum

**Terminal 1 — o "Raspberry":**

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
$CARMEN_HOME/data/pi_nit/venv/bin/python3 pi_nit_server.py \
    --dummy --bind 127.0.0.1 --batch-size 1
```

**Terminal 2 — o teste de enlace:**

```bash
$CARMEN_HOME/bin/pi_nit_link_test 127.0.0.1 -frames 30 -fps 15
```

```
frame    29 | rtt   65.2 ms | hailo  20.0 ms | fila  23.1 ms | 1 deteccao(oes)
        classe 0 conf 0.99 [256,80 -> 384,496]

enviados 30 | recebidos 29 | 14.5 fps de retorno
```

O `hailo 20.0 ms` é o atraso que o modo dummy simula de propósito; o `rtt` de
~65 ms é o loopback com JPEG dos dois lados.

### 0b — a cadeia IPC inteira, com 3 câmeras

Reinicie o servidor com o batch das 3 câmeras e a porta do viewer:

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
$CARMEN_HOME/data/pi_nit/venv/bin/python3 pi_nit_server.py \
    --dummy --bind 127.0.0.1 --batch-size 3 --viewer-port 5562
```

```bash
cd $CARMEN_HOME/bin
./central &
./param_daemon argos/carmen-argos.ini &

# as 3 "cameras": o mesmo video publicado nas mensagens 3, 4 e 5
for n in 3 4 5; do
    ./pi_nit_camera_publisher $n $CARMEN_HOME/data/pi_nit/pedestres.avi -fps 15 -loop 1 &
done

./pi_nit_client_driver intelbras1 3 intelbras2 4 intelbras3 5 \
    -pi_host 127.0.0.1 -fps 10 -show off
```

`Ctrl+C` no cliente fecha com o placar por câmera, que é o que interessa:

```
pi_nit: encerrando
  camera 3: enviados 149, recebidos 149
  camera 4: enviados 150, recebidos 149
  camera 5: enviados 149, recebidos 149
```

E a mensagem saindo no IPC, num outro terminal:

```bash
$CARMEN_HOME/bin/print_ipc_message neural_detector_message_5_name
```

```
int num_detected_objects : 1
bbox_i *detected_objects :  <{307, 96, 153, 479, 0.990, -1, 1}>
char *host :  " neural_detector_pi_nit@mobios"
```

> ⚠️ O `print_ipc_message` às vezes imprime os campos **sem os rótulos** (só
> `1`, `<{307, 96, ...}>`, o timestamp e o host, um por linha). É a mesma
> mensagem — não conclua que a câmera não está publicando só porque um
> `grep num_detected_objects` não casou.

Dois detalhes da saída acima, que **não** são defeito:

- `obj_id = -1` é o esperado. O padrão é `-coco_ids 0`, que publica a
  convenção do MOT (id COCO menos 1): pessoa `0` vira `-1`. Com
  `-coco_ids 1` sai o `0` do COCO.
- a caixa `307, 96, 153x479` está em pixels da imagem original de 768×576 —
  é a caixa fixa do dummy trazida de volta pelo letterbox. Ver a caixa
  **mudar** exige detecção de verdade (Teste 1).

O servidor avisa quando o batch não bate com o número de câmeras:

```
30.0 fps | 11.0 ms por imagem | 1.82 imagens por batch | 1.00 pessoa(s)/frame
batch de 3 configurado mas chegam 1.8 imagem(ns) por vez: o acelerador
processa 3 copias e descarta 1.
```

Com as 3 câmeras a 10 fps e o batch em 3, o servidor fecha o batch pela janela
de tempo antes das 3 imagens chegarem. No Hailo isso é desperdício de
inferência; no loopback é só o aviso.

---

## Teste 1 — tudo no PC, sem Raspberry (2 comandos)

Simula o Raspberry na sua própria máquina, usando o YOLO do
`multiple_object_tracker` no lugar do Hailo. As classes são as mesmas do
COCO, então o resultado é o mesmo que o Hailo vai devolver.

**Preparo (uma vez só):** o venv do Teste 0, mais o `ultralytics`:

```bash
$CARMEN_HOME/data/pi_nit/venv/bin/pip install ultralytics
```

**Terminal 1 — o "Raspberry" (na verdade a GPU do PC):**

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
$CARMEN_HOME/data/pi_nit/venv/bin/python3 pi_nit_server.py \
    --backend cpu --device 0 \
    --weights $CARMEN_HOME/data/pi_nit/weights/yolov8n.pt \
    --batch-size 3 --bind 127.0.0.1 --viewer-port 5562
```

**Terminal 2 — o vídeo em loop, como se fossem as 3 câmeras:**

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
$CARMEN_HOME/data/pi_nit/venv/bin/python3 tools/test_client.py \
    --host 127.0.0.1 \
    --video $CARMEN_HOME/data/pi_nit/pedestres.avi --loop \
    --simulate-cameras 3 --fps 15 --show
```

A janela abre com as caixas verdes em cima das pessoas, e o terminal mostra:

```
frame   144 | rtt   65.9 ms | hailo  10.8 ms | fila  36.9 ms | 6 deteccao(oes)
        classe 0 conf 0.84 [640,240 -> 684,323]
```

Os números entre colchetes são **pixels da imagem original**, não metros. A
posição em metros exige LiDAR e sai do `multiple_object_tracker` — veja o
Teste 3.

`q` ou `ESC` fecha a janela. `Ctrl+C` encerra.

### Ver as 3 câmeras lado a lado

Com o servidor do Terminal 1 ainda rodando, abra um **Terminal 3**:

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
python3 tools/pi_nit_viewer.py --host 127.0.0.1 --port 5562
```

Mosaico com as 3 câmeras, cada uma com timestamp, número de pessoas, tempo de
inferência e fps. É exatamente a tela que vai rodar no Raspberry.

### O que foi medido aqui (RTX 3050 Laptop, `yolov8n`)

```
44.3 fps | 10.2 ms por imagem | 3.00 imagens por batch | 5.11 pessoa(s)/frame
```

44 fps somados ÷ 3 câmeras ≈ **15 fps por câmera** — o alvo do plano.
O batch de 3 derrubou o tempo por imagem de ~94 ms para ~10 ms.

---

## Teste 2 — no Raspberry de verdade

Mesma coisa, trocando `127.0.0.1` pelo IP do Pi. O serviço já está rodando lá
(sobe sozinho no boot).

**Ligue a visualização no Pi, uma vez:**

```bash
ssh pi@192.168.1.20
sudo sed -i 's/^PI_NIT_VIEWER_PORT=.*/PI_NIT_VIEWER_PORT=5562/' /etc/pi_nit/pi_nit_server.conf
sudo systemctl restart pi_nit_server
exit
```

**No PC — vídeo em loop apontando para o Raspberry:**

```bash
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
python3 tools/test_client.py \
    --host 192.168.1.20 \
    --video $CARMEN_HOME/data/pi_nit/pedestres.avi --loop \
    --simulate-cameras 3 --fps 15 --show
```

**Na tela do próprio Raspberry (ou de qualquer PC da rede):**

```bash
python3 tools/pi_nit_viewer.py --host 192.168.1.20 --port 5562
```

Esperado com o Hailo-8L: `hailo` entre 15 e 20 ms, `rtt` entre 30 e 50 ms.

Sem janela nenhuma (só números), útil por SSH:

```bash
$CARMEN_HOME/bin/pi_nit_link_test 192.168.1.20 -frames 100 -fps 15
```

---

## Teste 2b — webcam do notebook, ao vivo no Raspberry

O teste mais rápido de todos: não precisa de `central`, `param_daemon`, câmera
da Iara nem log. Abre a webcam, manda para o Hailo e mostra as caixas.

```bash
$CARMEN_HOME/src/pi_nit/pi_nit_server/tools/webcam_ao_vivo.sh
```

Sem argumentos usa o Pi em `192.168.1.20`, a webcam `0` e 15 fps. Para mudar:

```bash
./webcam_ao_vivo.sh 192.168.1.136        # outro IP
./webcam_ao_vivo.sh 192.168.1.20 1 30    # webcam 1, 30 fps
```

Antes de abrir a janela ele confere o ping e, se conseguir entrar por ssh, se o
`pi_nit_server` está ativo — assim você não descobre que o serviço estava
parado só depois de olhar uma janela vazia. `q` ou ESC fecha.

Na faixa preta do topo: o IP, o fps de retorno, o tempo do Hailo e a ida e
volta da rede. Medido com o Pi em rede cabeada:

```
1 deteccao(oes) | pessoa conf 0.85 | hailo 17.9 ms | rtt 62 ms
```

Se quiser a saída em texto em vez da janela, é o `test_client.py` direto, que é
o que o script chama:

```bash
$CARMEN_HOME/data/pi_nit/venv/bin/python3 \
    $CARMEN_HOME/src/pi_nit/pi_nit_server/tools/test_client.py \
    --host 192.168.1.20 --camera 0 --fps 15 --frames 20
```

---

## Teste 3 — com o CARMEN de verdade (log tocando)

```bash
# terminal 1: central + playback do log com as câmeras
./central &
./playback /dados/<seu_log>.txt

# terminal 2: o módulo, apontando para o Raspberry
./pi_nit_client_driver intelbras1 3 intelbras2 4 intelbras3 5 \
    -pi_host 192.168.1.20 -fps 15 -show on

# terminal 3: confirmar que a mensagem está saindo (caixa 2D em pixels)
$CARMEN_HOME/bin/print_ipc_message neural_detector_message_3_name
```

### Para ver a posição de cada pessoa em metros

O pi_nit publica a caixa 2D — igual ao `neural_detector` do NIT, que também
**não** dá metros. Quem funde com o LiDAR e produz a posição 3D é o
`multiple_object_tracker`, rodando em cima da mesma mensagem:

```bash
# terminal 4: a fusão com LiDAR (mesmo comando de sempre)
./multiple_object_tracker intelbras1 3 -lidar 16 -detect_always 1

# terminal 5: a coordenada em metros
$CARMEN_HOME/bin/print_ipc_message carmen_moving_objects_point_clouds_message_0_name
```

Dois detalhes que fazem perder tempo:

- A saída do MOT é indexada pela **posição do argumento**, não pelo número da
  câmera: com uma câmera só, mesmo sendo a câmera 3, é `..._message_0_name`.
- **Sem LiDAR não há metros.** Nos testes 1 e 2 (vídeo em loop) não existe
  LiDAR, então lá você vê só a caixa 2D — é o esperado, não é defeito.

Detalhes e os campos da mensagem 3D no [README](README.md#de-onde-vem-a-coordenada-de-cada-pessoa-em-metros).

---

## Teste 4 — no playback da IARA, pelo process control

É o teste que vale, porque exercita o MOT de verdade: câmera do log, LiDAR,
`globalpos` e as telas.

```bash
cd $CARMEN_HOME/bin
./proccontrol ../data/iara_lcad/process_new/process-playback_iara.ini
```

O `.ini` já traz as duas receitas comentadas. Ligue/desligue os módulos na
janela do `proccontrol_gui`:

**Receita A — comparar as duas redes na mesma câmera**

| módulo | estado | o que faz |
|---|---|---|
| `ND` | 1 | a rede python alimenta o MOT |
| `Camera1` | 1 | tela do `camera_viewer`, com as caixas de quem está publicando |
| `pi_sim` | 1 | simulador do Raspberry no próprio PC |
| `pi_nit_show` | 1 | janela do pi_nit (`-publish 0`: mostra, não publica) |
| `pi_nit_local` | **0** | senão viram dois clientes no mesmo servidor |

**Receita B — o pi_nit alimentando o MOT**

`ND=0`, `pi_sim=1`, `pi_nit_local=1`, `MOT=1`, `Camera1=1`.

Em qualquer uma das duas, o `pi_nit_view` pode ficar ligado: ele lê direto do
servidor (porta 5562) e mostra o nome da classe, o tempo de inferência e o fps,
sem passar pelo IPC.

> As linhas do pi_nit no `.ini` já vão com `-ignore_bottom 0.13`, que apaga a
> faixa do capô antes de mandar para a rede. Sem isso o capô vira uma detecção
> de **carro** parada na base da imagem, em quase todo frame. Veja
> [O capô do carro vira detecção](README.md#o-capô-do-carro-vira-detecção).

### Pular o playback para onde tem gente

O `-message` do `playback_control` **não é velocidade** — é o intervalo do log,
em segundos (`logger/playback_interface.c:101`). E dá para mandar o comando
para o `playback` que já está rodando, sem clicar em nada:

```bash
./playback_control -message 't 174 : 180' -autostart on -speed 1.0
```

Isso vale ouro porque **os logs da IARA quase não têm pedestre**. No
`iara_20260730-3` (4 minutos, 3385 imagens de câmera), varrendo tudo com o
mesmo detector e o mesmo corte de 0,40:

| trecho | duração | pessoas | conf máx |
|---|---|---|---|
| `t 49 : 52` | 1,7 s | 1 | 0,50 |
| `t 59 : 62` | 3,0 s | 1 | 0,49 |
| `t 65 : 68` | 1,8 s | 1 | 0,60 |
| `t 87 : 90` | 2,0 s | 1 | 0,63 |
| `t 113 : 116` | 1,2 s | 1 | 0,54 |
| `t 168 : 170` | 1,6 s | 1 | 0,48 |
| **`t 174 : 180`** | 1,8 s | 1 | **0,78** |

29 frames com pessoa em 1129 amostrados. **Carro tem de sobra** — no trecho
`t 60 : 130` saíram 354 caixas de `Car` e 18 de `truck`. Se a tela ficar vazia,
a primeira coisa a checar é se o trecho do log tem alguém, não o detector.

A varredura sai de [`tools/acha_objetos_no_log.py`](pi_nit_server/tools/acha_objetos_no_log.py):

```bash
$CARMEN_HOME/data/pi_nit/venv/bin/python3 \
    $CARMEN_HOME/src/pi_nit/pi_nit_server/tools/acha_objetos_no_log.py \
    /dados/logs_iara/iara_20260730-3 --classes 0,2
```

```
trecho para o playback_control        dur  objetos  conf max   classes
  t     35 : 57                   22.7s        4      0.84   carro, pessoa
  t     61 : 77                   16.0s        5      0.90   carro, pessoa
  t    165 : 181                  16.0s        6      0.90   carro, pessoa
```

Cada linha já está no formato do `-message`.

---

## Quando não funciona

Teste **em ordem** — cada passo elimina uma camada:

```bash
# 1. o Raspberry responde? (sem IA, sem câmera, só rede)
ssh pi@192.168.1.20 'sudo systemctl stop pi_nit_server'
ssh pi@192.168.1.20 'cd /opt/pi_nit/app && sudo -u pi_nit ../venv/bin/python3 pi_nit_server.py --dummy'
$CARMEN_HOME/bin/pi_nit_link_test 192.168.1.20 -frames 30

# 2. o Hailo está vivo?
ssh pi@192.168.1.20 'hailortcli fw-control identify'

# 3. o serviço está de pé?
ssh pi@192.168.1.20 'journalctl -u pi_nit_server -n 50'
```

| Sintoma | Causa provável |
|---|---|
| `enviados N, recebidos 0` | serviço parado no Pi, ou porta 5560/5561 bloqueada |
| `descartados no envio` alto | o Pi não acompanha — baixe o `--fps` ou troque para `yolov8n` |
| `fila` alta, `hailo` baixo | decodificação de JPEG pesada; baixe `-jpeg_quality` |
| `rtt` alto, `hailo` baixo | rede (Wi-Fi? switch saturado?) |
| Detecta no PC mas não no Pi | HEF errado — precisa ser a versão `hailo8l`, não `hailo8` |
