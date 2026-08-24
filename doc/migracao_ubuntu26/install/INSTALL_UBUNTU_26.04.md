# Instalando o CARMEN LCAD no Ubuntu 26.04 LTS ("resolute")

Roteiro completo, do `git clone` até o build passando, para uma máquina nova.
É só seguir na ordem — cada passo pode ser copiado e colado.

**A boa notícia:** este tutorial é bem mais curto que os de 16.04/20.04. Naquela
época era preciso compilar Boost, OpenCV, PCL (com patches manuais!), g2o, dlib e
FANN da fonte. No 26.04 tudo isso tem pacote `apt` — sobrou **uma única**
biblioteca sem pacote (o GtkGLExt, passo 7).

Ambiente validado: Ubuntu 26.04 LTS, gcc/g++ 15.2.0, CMake 4.2.3, Python 3.14.

> Se algo der errado, o motivo de cada mudança está no [`CHANGELOG.md`](../CHANGELOG.md),
> e o detalhe de cada passo nos arquivos `00_` a `05_` desta pasta. As pendências
> conhecidas estão em [`05_pendencias_conhecidas.md`](05_pendencias_conhecidas.md).

---

## 1. Clonar o repositório

```bash
git clone https://github.com/LCAD-UFES/carmen_lcad.git ~/carmen_lcad
```

## 2. Variáveis de ambiente

Adicione no fim do `~/.bashrc`:

```bash
#CARMEN
export CARMEN_HOME=~/carmen_lcad
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig
export LD_LIBRARY_PATH=$CARMEN_HOME/lib:$LD_LIBRARY_PATH:/usr/local/lib:/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu/:/usr/lib/libkml
export PATH=$PATH:$CARMEN_HOME/bin

#OpenJaus (as .so ficam dentro do próprio repo, não em /usr/local)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CARMEN_HOME/sharedlib/OpenJAUS/libopenJaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/libjaus/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojTorc/lib:$CARMEN_HOME/sharedlib/OpenJAUS/ojIARASim/lib

#Darknet (usado pelos módulos de detecção neural / yolo)
export DARKNET_HOME=$CARMEN_HOME/sharedlib/darknet
export LD_LIBRARY_PATH=$DARKNET_HOME/lib:$LD_LIBRARY_PATH

#MAE (só necessário para o módulo tracker/MAE; descomente se instalar o MAE)
#export MAEHOME=~/MAE
#export PATH=$PATH:$MAEHOME/bin
```

Recarregue e confira:

```bash
source ~/.bashrc
echo $CARMEN_HOME              # /home/<usuário>/carmen_lcad
ls $CARMEN_HOME/src/Makefile.conf
```

> ⚠️ Se esta máquina tiver **outro repositório do LCAD** (um fork do CARMEN), os
> dois geram bibliotecas com os mesmos nomes (`libglobal.so`, `libipc.a`, ...).
> Quem aparecer primeiro no `LD_LIBRARY_PATH` ganha em tempo de execução. O bloco
> acima põe `$CARMEN_HOME/lib` **na frente** de propósito. Detalhes em
> [`00_ambiente_e_bashrc.md`](00_ambiente_e_bashrc.md).

## 3. Ferramentas de build

```bash
sudo apt-get update

sudo apt-get install -y \
    build-essential gcc g++ make cmake cmake-curses-gui \
    git pkg-config doxygen byacc flex swig perl \
    wget tcsh vim gnuplot-qt plocate
```

Confira as versões:

```bash
gcc --version      # precisa ser 15.x
cmake --version    # 4.x
perl --version     # o src/configure é um script Perl
```

> ⚠️ **O `gcc` padrão precisa ser o gcc-15.** Todas as bibliotecas do apt no 26.04
> foram compiladas com gcc-15. Se alguém tiver fixado um gcc antigo via
> `update-alternatives` (comum por causa de CUDA), o build compila mas **quebra no
> link**, com `undefined reference to ...@GLIBCXX_3.4.30`. Isso não é bug do
> CARMEN — é ABI incompatível. Confira com `update-alternatives --list gcc`; a
> solução está em [`01_ferramentas_de_build.md`](01_ferramentas_de_build.md).

## 4. Boost

**Diferente do tutorial antigo: NÃO compile o Boost na mão e NÃO use o prefixo
`/usr/local/carmen_boost`.** Os 28 Makefiles que apontavam para lá foram
corrigidos; hoje o build usa o Boost do sistema.

```bash
sudo apt-get install -y libboost-all-dev
```

## 5. Bibliotecas que antes eram compiladas na mão

Nenhuma delas precisa mais de build manual:

```bash
sudo apt-get install -y \
    libopencv-dev \
    libpcl-dev \
    libg2o-dev \
    libdlib-dev \
    libeigen3-dev \
    libgsl-dev \
    libbullet-dev \
    libsuitesparse-dev \
    liblapacke-dev \
    libhdf5-dev \
    libflann-dev \
    libqhull-dev \
    libvtk9-dev
```

Versões no 26.04: OpenCV 4.10, PCL 1.15.1, g2o 0~20230806, Eigen 3.4, GSL 2.8,
Bullet 3.24, SuiteSparse 7.12, VTK 9.5.2.

## 6. Demais pacotes apt

```bash
sudo apt-get install -y \
    libgtk2.0-dev libgtk-3-dev libgtkmm-2.4-dev qtbase5-dev \
    libzmq3-dev \
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

## 7. GtkGLExt — a única lib sem pacote

Este é **o único passo que ainda exige compilar da fonte**, e o ponto onde o build
de uma máquina nova costuma parar, com:

```
gtk/gtkgl.h: No such file or directory
```

O `pkg-config gtkglext-1.0` é usado pela GUI principal (`navigator_gui2`) e por
mais ~20 módulos (`mapper`, `road_mapper`, `localize_ackerman`,
`simulator_ackerman`, `obstacle_avoider`, ...). O projeto está morto upstream
desde ~2008 e não tem pacote em nenhuma versão recente do Ubuntu.

**Primeiro, confira se já não está instalado** (em várias máquinas do laboratório
ele veio de uma migração anterior):

```bash
pkg-config --modversion gtkglext-1.0     # esperado: 1.2.0
ls /usr/local/include/gtkglext-1.0/gtk/gtkgl.h
```

Se os dois comandos responderem, pule para o passo 8.

Se **não** estiver instalado, o roteiro abaixo foi executado do zero nesta máquina e
funciona. São o tarball oficial do GNOME (2010, a última release) mais os seis
patches de build que o pacote Debian mantido pela Deepin ainda carrega — a lib está
morta upstream desde ~2008, mas as distros que ainda a empacotam mantêm os patches
que a fazem compilar contra GTK2/glibc/Pango modernos.

**Pré-requisito:** os patches mexem em `configure.in`, então é preciso regerar o
`configure` com os autotools, que não entram na lista do passo 3:

```bash
sudo apt-get install -y autoconf automake libtool
```

**O build:**

```bash
mkdir -p ~/packages/gtkglext_build && cd ~/packages/gtkglext_build

curl -LO https://download.gnome.org/sources/gtkglext/1.2/gtkglext-1.2.0.tar.gz
# md5 esperado: 5c3240bfc1b21becd33ce35c5abe6f8d
mkdir -p patches && cd patches
for p in 01_fix_fontcache_nullderef.diff \
         02_fix_gtk-2.20_deprecated_symbols.diff \
         03_gdkglext-config-h-installation-path.diff \
         04_glibc2.27-ftbfs.diff \
         05_nopangox.diff \
         libGL.so.1.diff ; do
    curl -LO https://raw.githubusercontent.com/deepin-community/gtkglext/master/debian/patches/$p
done
cd ..

tar xzf gtkglext-1.2.0.tar.gz
cd gtkglext-1.2.0

for p in ../patches/*.diff; do patch -p1 < "$p"; done

autoreconf -fi                 # os patches mexem em configure.in
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

O `configure` deve terminar com `multihead support: yes` e `OpenGL LIBS: -lGLU -lGL`.
Os avisos de `automake` sobre `INCLUDES` e `configure.in` são cosméticos — é código de
2006 sendo processado por autotools de 2026.

**Confira depois de instalar:**

```bash
pkg-config --modversion gtkglext-1.0     # 1.2.0
pkg-config --cflags gtkglext-1.0         # -I/usr/local/include/gtkglext-1.0 ...
ls /usr/local/include/gtkglext-1.0/gtk/gtkgl.h
ls /usr/local/include/gtkglext-1.0/gdkglext-config.h
```

Para o que cada patch serve:

| patch | o que conserta |
|---|---|
| `01_fix_fontcache_nullderef.diff` | *null deref* na função de cache de fonte |
| `02_fix_gtk-2.20_deprecated_symbols.diff` | símbolos do GTK2 removidos desde a 2.20 |
| `03_gdkglext-config-h-installation-path.diff` | põe o `gdkglext-config.h` no `includedir` |
| `04_glibc2.27-ftbfs.diff` | quebra de build com glibc >= 2.27 |
| `05_nopangox.diff` | tira a dependência de **PangoX**, que não existe mais no Pango atual (só sobrou PangoXft). Sem ele nem compila |
| `libGL.so.1.diff` | ajuste do link contra a `libGL` |

> O `series` do pacote Debian deixa o `03` comentado; aqui ele **é aplicado**. É o que faz
> o `gdkglext-config.h` ser instalado em `/usr/local/include/gtkglext-1.0/` em vez de
> dentro do `libdir` — que é onde o `.pc` gerado manda procurar
> (`Cflags: -I${includedir}/gtkglext-1.0`). Sem o `03`, quem incluir `gdkglext-config.h`
> não acha o header.

O header vai para `/usr/local/include/gtkglext-1.0/gtk/gtkgl.h` e o `.pc` gerado aponta
para lá, então **nenhum Makefile do CARMEN precisa mudar** — todos usam
`pkg-config gtkglext-1.0`.

> Verificado em 2026-08-20: baixado, aplicado, `autoreconf`, `configure` e
> `make -j` do zero, e a instalação conferida num `DESTDIR` local (sem `sudo`),
> que reproduziu exatamente o layout já presente em `/usr/local`. O único passo
> não reexecutado foi o `sudo make install`, porque a lib já estava instalada
> nesta máquina.

## 8. `configure`

Rodar **uma vez**, dentro de `src/`:

```bash
cd $CARMEN_HOME/src
./configure --nojava --nocuda --noqt3
```

Responda **Enter em todas as perguntas** (os defaults são os corretos):

| Pergunta | Resposta | Por quê |
|---|---|---|
| `Should the C++ tools be installed [Y/n]` | Enter (Y) | O build usa C++ em quase tudo |
| `Should Python Bindings be installed [y/N]` | Enter (N) | Bindings SWIG da era Python 2; só dão erro |
| `Should the old laser server be used [y/N]` | Enter (N) | `sensors/laser_new` é o driver atual |
| `Install path [/usr/local/]` | Enter | Não é usado por nenhuma regra do build |
| `Robot numbers [*]` | Enter | Só decide quais módulos de base compilam |

As flags: `--nojava` (bindings pré-Java 8, não usadas), `--nocuda` (ver passo 10),
`--noqt3` (Qt3 morreu há mais de uma década).

Isso gera o `src/Makefile.vars` (**não** versionado). É esperado ver
`NO_TCPD = 1` — é só o controle de acesso opcional do IPC, não bloqueia nada.

## 9. Compilar

```bash
cd $CARMEN_HOME/src
make -j$(nproc)
```

O build termina com `Done making binaries...` e código de saída **0**, gerando
~657 binários em `bin/`, ~138 arquivos em `lib/` e ~279 headers em
`include/carmen/`.

Se o `gcc` padrão não for o 15 (passo 3), use:

```bash
make -C $CARMEN_HOME/src CC=gcc-15 CXX=g++-15 -j$(nproc)
```

### Quatro armadilhas do build

⚠️ **Nunca rode dois `make` simultâneos na mesma árvore.** Não há lock: um
processo compila o `.o` enquanto o outro linka o arquivo pela metade, gerando
erros de link não-determinísticos que parecem bug de código. Antes de investigar:
`ps -eo args | grep '[m]ake -j'`.

⚠️ **O `make` do topo não é tolerante a falha.** O `RECURSE` do `src/Makefile` é
um `for` do shell com `exit -1` embutido: o primeiro módulo que falhar aborta
tudo, e nem `make -k` ajuda. Para pular um módulo e testar o resto, tire o nome
dele da lista `PACKAGES` em `src/Makefile`.

⚠️ **`undefined reference` para algo que está claramente no código**: quase sempre
é `.o` velho de uma tentativa anterior.

```bash
cd $CARMEN_HOME/src/<modulo> && rm -f *.o *.a Makefile.depend && make
```

⚠️ **Nunca rode `make` num módulo que está fora da lista `PACKAGES`** do
`src/Makefile`. O alvo `export` de cada módulo **repõe os symlinks de
`include/carmen/`** apontando para os headers dele. Como os módulos fora do
`PACKAGES` estão fora justamente por terem código desatualizado, os headers deles
são incompatíveis — e um `make` inocente ali sequestra headers públicos e quebra
a compilação de módulos que não têm nada a ver.

Aconteceu de verdade: um `make` em `src/path_planner2` repontou
`rddf_{messages,interface,util,index}.h` de `src/rddf/` para `src/path_planner2/`,
e todo módulo que usa rddf parou de compilar com
`'carmen_ackerman_traj_point_t' does not name a type`.

Para conferir se algum header foi sequestrado:

```bash
cd $CARMEN_HOME
for f in include/carmen/*; do
  t=$(readlink "$f"); [ -n "$t" ] && echo "$t"
done | sed 's|.*/src/||; s|/.*||' | sort -u
```

Todo nome que aparecer precisa estar no `PACKAGES` do `src/Makefile`. Para
consertar, refaça o symlink para o módulo correto:

```bash
ln -sf $CARMEN_HOME/src/<modulo_certo>/<header>.h $CARMEN_HOME/include/carmen/<header>.h
```

## 10. Opcional: hardware, CUDA e ROS

Só instale o que a máquina for realmente usar.

- **CUDA** — o `configure` acima foi rodado com `--nocuda`. Antes de ligar,
  confirme que existe um CUDA Toolkit compatível com o **GCC 15**. Toolkits
  antigos recusam compiladores novos, e a saída típica é instalar um `g++` mais
  velho só para o `nvcc` — o que interage com o aviso de ABI do passo 3: se o
  `gcc` *padrão* virar o antigo, o link contra as libs do apt quebra.
- **Kvaser CAN SDK** — https://www.kvaser.com/download/, depois `make && sudo make
  install`. Sem ele o `configure` marca `NO_CANLIB = 1` e os módulos de CAN ficam
  de fora.
- **FLIR/PointGrey Flycapture 2** — necessário só para `bumblebee_basic` e
  `utilities/list_camera_ids`:
  ```bash
  git clone https://github.com/RhobanDeps/flycapture.git
  cd flycapture && sudo sh install_flycapture.sh
  ```
- **libwnn**:
  ```bash
  git clone http://github.com/filipemtz/libwnn
  cd libwnn && mkdir build && cd build
  cmake .. && make -j$(nproc) && sudo make install
  ```
  (o `-D OpenCV_DIR=...` do tutorial antigo não é mais necessário — o OpenCV vem
  do apt.)
- **Ouster SDK antigo (`ouster_example`)** — para
  `sensors/ouster_sdk_driver_5-2`. O CMake 4 do 26.04 recusa
  `cmake_minimum_required` < 3.5: precisa de `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`
  no configure **e** de subir o `cmake_minimum_required` dentro de
  `cmake/VersionGen.cmake` (que roda como `cmake -P` separado e não herda a flag).
- **ROS 1 Noetic** — só para o módulo `src/sc_lio_sam` (SLAM 3D). Não há pacote
  do Noetic para o 26.04; é preciso compilar da fonte. O build do módulo é
  separado (`catkin_make`) e está documentado no
  [`README.md` dele](../../../src/sc_lio_sam/README.md).

  > Instale o ROS **por último**: ele reescreve pacotes de que o CARMEN depende.

## 11. Python

O Python do 26.04 é o 3.14 e bloqueia `pip install` fora de venv (PEP 668,
`error: externally-managed-environment`). Prefira pacotes `python3-*` do apt;
quando não houver, crie um venv explícito.

---

## Conferindo que funcionou

Os `.ini` de parâmetros ficam em `src/`, e os binários rodam a partir de `bin/`
(os caminhos dentro dos `.ini` são relativos a `bin/`):

```bash
cd $CARMEN_HOME/bin
./central &
./param_daemon ../src/carmen-ford-escape-sensorbox.ini
```

Ou, para subir um conjunto de módulos de uma vez — é assim que o sistema roda de
verdade:

```bash
cd $CARMEN_HOME/bin
./proccontrol process-volta_da_ufes_playback_viewer_3D.ini
```

## Se travar

| Sintoma | Causa provável | Onde está a resposta |
|---|---|---|
| `undefined reference ...@GLIBCXX_*` | `gcc` padrão não é o 15 | passo 3 / [`01_`](01_ferramentas_de_build.md) |
| `gtk/gtkgl.h: No such file` | GtkGLExt não instalado | passo 7 / [`04_`](04_bibliotecas_sem_pacote_apt.md) |
| `cannot find -lboost_<algo>` | Makefile que a varredura não pegou | [`02_boost.md`](02_boost.md) |
| `#error C++ standard too low` | Módulo em C++11/14 usando PCL 1.15 | [`03_`](03_opencv_pcl_g2o_dlib_eigen.md) |
| `opencv2/*.hpp: No such file` | Módulo linka OpenCV sem pedir `--cflags` | [`03_`](03_opencv_pcl_g2o_dlib_eigen.md) |
| `cannot find -lnsl` | Falta `libnsl-dev` | passo 6 |
| Módulo não compila e não está na lista | Pode ser pendência conhecida | [`05_pendencias_conhecidas.md`](05_pendencias_conhecidas.md) |
