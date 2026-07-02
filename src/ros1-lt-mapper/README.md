# Tutorial — Pipeline SC-LIO-SAM + CARMEN IPC Bridge + LT-mapper

Fluxo completo para gravar uma sessão com o Hesai XT-32 + Xsens via CARMEN, rodar o SC-LIO-SAM em Docker (ROS1 Noetic) e preparar os dados para o LT-mapper.

---

## Visão geral do fluxo

1. Instalar/garantir ROS1 Noetic no **host** (fora do Docker) — necessário para compilar e rodar o `carmen_ipc_bridge_noetic`.
2. Buildar a imagem Docker (`sc-lio-sam` / `lt-mapper`).
3. Subir o container `ros1_sclio` (ele já inicia o `roscore` via `entrypoint.sh`).
4. No host, rodar `run.sh` para carregar o ambiente ROS1 do workspace do bridge (`ros_ipc_carmen`), apontando para o `roscore` do container.
5. Só então subir a ponte: `roslaunch carmen_ipc_bridge carmen_ipc_bridge.launch`.
6. Rodar os sensores/log do CARMEN (Hesai + Xsens) para o LIO-SAM gerar e salvar o mapa da sessão.
7. Repetir para as demais sessões necessárias.
8. Subir o container `ros1_ltmapper` para rodar o LT-mapper sobre as sessões salvas.

---

## 1. Instalar o ROS1 Noetic no host

O `carmen_ipc_bridge_noetic` roda **fora** do container (ele é quem faz a ponte entre o IPC do CARMEN, que também roda no host, e o ROS1 do container via `ROS_MASTER_URI=http://localhost:11311`). Por isso o Noetic precisa estar instalado nativamente:

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y python3-catkin-tools python3-rosdep

sudo rosdep init   # se ainda não foi feito
rosdep update
```

Confirme:

```bash
source /opt/ros/noetic/setup.bash
roscore --help
```

---

## 2. Buildar a imagem Docker

Na raiz do `ros1-lt-mapper` (onde estão o `Dockerfile` e o `docker-compose.yml`):

```bash
cd ~/carmen_lcad/src/ros1-lt-mapper
docker compose build
```

Isso builda a imagem usada tanto pelo serviço `ros1_sclio` quanto pelo `ros1_ltmapper` (ambos usam `build: .`), instalando ROS1 Noetic, GTSAM, ceres-solver, SC-LIO-SAM e LT-mapper dentro do container.

---

## 3. Buildar o workspace do bridge (`ros_ipc_carmen`)

Antes de rodar o `run.sh`, o workspace do bridge precisa estar compilado:

```bash
cd ~/carmen_lcad/src/ros1-lt-mapper/ros_ipc_carmen
source /opt/ros/noetic/setup.bash
catkin build carmen_ipc_bridge
```

> Confirme que o pacote se chama `carmen_ipc_bridge` no `package.xml` de `src/carmen_ipc_bridge_noetic` — o `run.sh` espera o `devel/setup.bash` gerado aqui, na mesma pasta de `ros_ipc_carmen`.

---

## 4. Subir o container do SC-LIO-SAM

Defina a sessão que vai ser gravada e suba o serviço `ros1_sclio`:

```bash
cd ~/carmen_lcad/src/ros1-lt-mapper
SESSION_NAME=session1 docker compose up ros1_sclio
```

Isso:
- monta `./lt-mapper/sessions/in/session1` em `/home/user/Desktop/scliosam/data`;
- sobe o `roscore` em `localhost:11311` (com lock em `/shared/roscore.lock`, então o `ros1_ltmapper` depois reaproveita o mesmo master se já estiver de pé);
- executa `roslaunch lio_sam run.launch`.

Deixe esse terminal rodando.

---

## 5. Carregar o ambiente do bridge no host

Em **outro terminal**, no host:

```bash
cd ~/carmen_lcad/src/ros1-lt-mapper/ros_ipc_carmen
./run.sh
```

O `run.sh` faz:
- `source /opt/ros/noetic/setup.bash`
- `source ./devel/setup.bash`
- exporta `ROS_MASTER_URI=http://localhost:11311` (o mesmo master que o container `ros1_sclio` subiu, já que o container roda com `network_mode: host`)
- exporta `ROS_IP` automaticamente

Ele te deixa dentro de um `bash` com o ambiente pronto.

---

## 6. Subir a ponte CARMEN → ROS1 (só depois do passo 4 estar de pé)

**Importante:** só rode isso depois que o `roscore` do `ros1_sclio` já estiver ativo (passo 4), senão o `roslaunch` do bridge não encontra o master.

No mesmo terminal do `run.sh`:

```bash
roslaunch carmen_ipc_bridge carmen_ipc_bridge.launch
```

Isso conecta no IPC do CARMEN (Hesai XT-32 + Xsens IMU) e publica os tópicos ROS1 que o `lio_sam` (rodando no container) está esperando.

---

## 7. Gravar a sessão

Com o bridge publicando e o LIO-SAM rodando dentro do container, **rode os módulos CARMEN dos sensores** (ou reproduza o log) normalmente, do jeito que você já usa no `carmen_lcad`, para o Hesai e o Xsens começarem a publicar via IPC.

O LIO-SAM vai consumir isso em tempo real e salvar o mapa/poses da sessão em:

```
./lt-mapper/sessions/in/session1/
```

(pasta montada no volume, conforme definido no `docker-compose.yml`).

Ao terminar a gravação, encerre o `roslaunch` do CARMEN, o `carmen_ipc_bridge` e o container `ros1_sclio` (Ctrl+C).

---

## 8. Repita para outras sessões

Para gravar `session2` (ou outra), repita os passos 4 a 7 trocando o `SESSION_NAME`:

```bash
SESSION_NAME=session2 docker compose up ros1_sclio
```

> Lembre-se: cada sessão precisa passar pelo bridge + sensores rodando de novo — o LIO-SAM só grava o que realmente foi publicado nos tópicos ROS1 durante a execução.

---

## 9. Rodar o LT-mapper sobre as sessões salvas

Depois de ter todas as sessões necessárias em `./lt-mapper/sessions/in/`:

```bash
docker compose up ros1_ltmapper
```

Isso sobe `roslaunch ltslam run.launch`, usando:
- `./lt-mapper/config` → configs do LT-mapper
- `./lt-mapper/sessions` → sessões de entrada e saída (`in/` e `out/`)

Se o `ros1_sclio` ainda estiver de pé com o `roscore` ativo, o `ros1_ltmapper` reaproveita o mesmo master (graças ao `flock` no `entrypoint.sh`); caso contrário, ele sobe o seu próprio `roscore`.

---

## Checklist rápido

- [ ] ROS1 Noetic instalado no host
- [ ] `catkin build carmen_ipc_bridge` feito em `ros_ipc_carmen`
- [ ] `docker compose build` feito
- [ ] `SESSION_NAME=<nome> docker compose up ros1_sclio` rodando
- [ ] `./run.sh` carregado em outro terminal
- [ ] `roslaunch carmen_ipc_bridge carmen_ipc_bridge.launch` só depois do roscore estar de pé
- [ ] Sensores CARMEN (Hesai + Xsens) publicando
- [ ] Sessão salva em `lt-mapper/sessions/in/<nome>`
- [ ] Repetir para todas as sessões necessárias
- [ ] `docker compose up ros1_ltmapper` para rodar o LT-mapper