# 4. Demais pacotes apt e as bibliotecas sem pacote

## 4.1 Pacotes GTK/X11/áudio/utilitários (todos ainda existem no apt do 26.04)

```bash
sudo apt-get install -y \
    libgtk2.0-dev libgtk-3-dev qtbase5-dev \
    libimlib2-dev imagemagick libmagick++-dev \
    libwrap0-dev \
    libncurses-dev \
    freeglut3-dev libglew-dev \
    libcurl4-openssl-dev \
    libkml-dev liburiparser-dev \
    libusb-1.0-0-dev libusb-dev \
    libxi-dev libxmu-dev \
    libforms-dev \
    libgflags-dev libespeak-dev libfftw3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libavutil-dev libtbb-dev \
    libjpeg-dev libpng-dev libpng++-dev libtiff-dev \
    libgstreamer-plugins-base1.0-dev \
    python3-numpy python3-dev python3-pip python3-wheel \
    mpi-default-dev openmpi-bin openmpi-common \
    libgtest-dev libudev-dev \
    libasound2-dev mpg123 portaudio19-dev \
    libjsoncpp-dev libfann-dev libnsl-dev libnotify-dev \
    libpcap-dev
```

Trocas em relação ao tutorial de 16.04/20.04 (detalhes em `../AUDIT_DEPENDENCIAS.md`):

| Tutorial antigo | Ubuntu 26.04 |
|---|---|
| `libncurses5-dev` | `libncurses-dev` |
| `libcurl4-nss-dev` | `libcurl4-openssl-dev` |
| `libqt4-dev`, `qt4-qmake` | `qtbase5-dev` (só `global_graphics_qt`/`car_panel_gui` usam Qt) |
| `libgtkglext1-dev`, `libgtkglextmm-x11-1.2-dev` | **NOT-FOUND** — ver seção 4.3 |
| `libglade2-dev`, `libglademm-2.4-dev` | **NOT-FOUND** — o código já usa `GtkBuilder`, ver `../CHANGELOG.md` |
| `libdc1394-22-dev` | `libdc1394-dev`/`libdc1394-utils` (só para câmera FireWire) |
| `libjasper-dev` | Só era necessário para compilar o OpenCV 3 na mão — irrelevante agora |
| `libvtk5-dev`/`libvtk6-dev` | `libvtk9-dev` (já vem como dependência do `libpcl-dev`) |
| `python-numpy`, `python-dev` (Python 2) | `python3-numpy`, `python3-dev` — Python 2 não existe mais |

## 4.2 `libnsl` (o `param_daemon` linka `-lnsl`)

`libnsl` foi separada da glibc; sem `libnsl-dev` o link do `param_daemon` falha com
`cannot find -lnsl`. Já está na lista acima.

## 4.3 GtkGLExt — não existe em pacote nenhum, precisa ser compilado

`pkg-config gtkglext-1.0` é usado pela GUI principal (`navigator_gui2`) e por mais ~20
módulos (`mapper`, `road_mapper`, `localize_ackerman`, `simulator_ackerman`,
`obstacle_avoider`, ...). O projeto está morto upstream desde ~2008 e não tem pacote no
Ubuntu 26.04.

A rota que funcionou (já validada na migração de um fork deste mesmo código-base) é compilar da fonte,
a partir do tarball oficial do GNOME + os patches de build que o pacote Debian da Deepin ainda
mantém, seguido de `sudo make install`. Depois disso o header vai para
`/usr/local/include/gtkglext-1.0/gtk/gtkgl.h` e o `.pc` gerado aponta para lá — **nenhum
Makefile do CARMEN precisa mudar**, já que todos usam `pkg-config gtkglext-1.0`.

Nesta máquina o GtkGLExt **já estava instalado** (veio da migração de outro repositório do LCAD), então o build do
CARMEN encontrou tudo pronto. Para conferir numa máquina nova:

```bash
pkg-config --modversion gtkglext-1.0   # esperado: 1.2.0
ls /usr/local/include/gtkglext-1.0/gtk/gtkgl.h
```

Se não estiver instalado, é aqui que o build vai parar com `gtk/gtkgl.h: No such file or
directory`. **A sequência completa de comandos está no passo 7 do
[`INSTALL_UBUNTU_26.04.md`](INSTALL_UBUNTU_26.04.md#7-gtkglext--a-única-lib-sem-pacote)** —
tarball do GNOME + os seis patches do pacote Debian da Deepin, `autoreconf -fi`,
`./configure --prefix=/usr/local`, `make`, `sudo make install`, `sudo ldconfig`. Foi
executada e verificada do zero em 2026-08-20.

## 4.4 SDKs de hardware (proprietários / sem pacote)

Iguais ao tutorial original — só instalar o que a máquina realmente for usar:

- **Kvaser CAN SDK (`linuxcan`)** — https://www.kvaser.com/download/, `make && sudo make
  install`. Sem ele o `configure` marca `NO_CANLIB = 1` e os módulos de CAN ficam de fora.
- **FLIR/PointGrey Flycapture 2** — `git clone https://github.com/RhobanDeps/flycapture.git
  && cd flycapture && sudo sh install_flycapture.sh`. Necessário só para `bumblebee_basic`
  (câmera Bumblebee2) e `utilities/list_camera_ids`.
- **Ouster SDK antigo (`ouster_example`)** — necessário para `sensors/ouster_sdk_driver_5-2`.
  Atenção: o CMake 4 do 26.04 recusa `cmake_minimum_required` < 3.5 — precisa de
  `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` no configure **e** de subir o
  `cmake_minimum_required` dentro de `cmake/VersionGen.cmake` (que roda como `cmake -P`
  separado e não herda a flag).
- **libwnn** — `git clone http://github.com/filipemtz/libwnn && cd libwnn && mkdir build &&
  cd build && cmake .. && make -j$(nproc) && sudo make install`. Como o OpenCV agora vem do
  apt, o `-D OpenCV_DIR=...` do tutorial antigo não é mais necessário.

## 4.5 CUDA (só se a máquina tiver GPU NVIDIA)

O `configure` foi rodado com `--nocuda` nesta migração. Antes de ligar CUDA, confirmar que
existe um CUDA Toolkit compatível com o **GCC 15** do 26.04 — toolkits antigos (o tutorial
falava de CUDA 9.1/11) costumam recusar compiladores muito novos, e a saída típica é instalar
um `g++` mais velho só para o `nvcc`, via `update-alternatives`. Isso interage com o aviso de
ABI do `01_ferramentas_de_build.md`: se o `gcc` *padrão* virar o antigo por causa do CUDA, o
link contra as libs do apt quebra.

## 4.6 Python

O Python do Ubuntu 26.04 é o 3.14 e bloqueia `pip install` fora de venv (PEP 668,
`error: externally-managed-environment`). Preferir pacotes `python3-*` do apt; quando não
houver, criar um venv explícito. Os Makefiles que fixavam `python3.5`/`3.6`/`3.8` foram
ajustados para usar `python3-config`/`pkg-config python3` (ver `../CHANGELOG.md`).

Próximo passo: `05_pendencias_conhecidas.md`.
