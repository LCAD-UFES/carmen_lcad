# 3. OpenCV, PCL, g2o, Eigen, GSL, VTK e afins

A maior mudança boa em relação ao tutorial de 16.04/20.04: **nenhuma dessas bibliotecas
precisa mais ser compilada na mão.** O tutorial antigo mandava compilar OpenCV 3.2 (e depois
4.5.5), PCL 1.8 (com patches manuais em vários arquivos!), g2o (clone do fork do LCAD), dlib,
FANN (do SourceForge). No Ubuntu 26.04 tudo isso tem pacote apt oficial:

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

Versões confirmadas com `apt-cache policy` nos repositórios reais do 26.04 "resolute" (lista
completa em `../AUDIT_DEPENDENCIAS.md`): OpenCV 4.10, PCL 1.15.1, g2o 0~20230806, Eigen 3.4,
GSL 2.8, Bullet 3.24, SuiteSparse 7.12, VTK 9.5.2.

## O que muda para o build do CARMEN

### OpenCV — só existe a versão 4

- Não existe mais o `opencv.pc` antigo, só `opencv4.pc`. Os ~200 Makefiles que chamavam
  `` `pkg-config --cflags opencv` `` foram trocados por
  `` `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv` `` (item 5 do
  `../CHANGELOG.md`) — funciona nas duas distros.
- Vários módulos linkavam OpenCV (`--libs`) mas nunca pediam os `-I` (`--cflags`): no 20.04
  os headers ficavam em `/usr/include/opencv2` (path padrão do compilador), no 26.04 ficam em
  `/usr/include/opencv4`, que **exige** `-I`. Corrigido nos 38 módulos afetados (item 8).
- A API C do OpenCV (`IplImage`, `cvLoadImage`, `CV_LOAD_IMAGE_COLOR`, ...) foi removida ou
  movida de header. Ver os itens de porte no `../CHANGELOG.md`.
- `initUndistortRectifyMap` e afins saíram do `imgproc` e ficam só no `calib3d` (item 9).

### PCL — headers em `/usr/include/pcl-1.15`, e exige C++17

- O tutorial antigo instalava PCL 1.8 em `/usr/local/include/pcl-1.8`; dezenas de Makefiles
  fixam esse caminho. Em vez de editar todos, `src/Makefile.conf` passou a detectar
  automaticamente qualquer `pcl-*` instalado (item 6).
- **PCL 1.15 exige `-std=c++17`** (o próprio `pcl_config.h` dá `#error C++ standard too low`).
  Todos os Makefiles que usam PCL e ainda estavam em `c++11`/`c++14` foram subidos (item 11).

### VTK — 9.5, e o sufixo de versão faz parte do `-l`

O VTK não publica `.pc` nem uma `.so` sem versão: a lib chama-se
`libvtkCommonCore-9.5.so`. Makefiles que linkavam `-lvtkRenderingCore-6.3` (ou `-lvtkCommon`,
nome da era VTK 5/6) não linkam mais. `src/Makefile.conf` agora detecta o sufixo real
(`VTK_LIB_SUFFIX`) — ver item 6 e as observações do `05_pendencias_conhecidas.md`.

### g2o / CSparse — o problema é o `cs.h`, não o g2o

Os `#include` de g2o no código já usam o prefixo (`"g2o/core/..."`) e o `libg2o-dev` instala
em `/usr/include/g2o`, que já é path padrão do gcc — **não precisa de `-I` para o g2o**. O que
precisa: o header `g2o/solvers/csparse/csparse_extension.h` faz `#include <cs.h>` sem prefixo.
Antes isso vinha do `EXTERNAL/csparse/` vendorizado (`-I/usr/local/include/EXTERNAL/csparse/`,
que não existe mais); no apt o `cs.h` vem do `libsuitesparse-dev`, em `/usr/include/suitesparse`.
Adicionado globalmente no `src/Makefile.conf` (item 6).

### Qt — Qt4 morreu, o pouco que o CARMEN usa foi para o Qt5

`src/global/global_graphics_qt.cpp` (usado por `car_panel_gui`) precisava de headers Qt4 em
`/usr/include/qt4` e `/usr/lib64/qt4`, que não existem mais. O código só usa
`QObject`/`QSocketNotifier`, que são idênticos no Qt5:

```bash
sudo apt-get install -y qtbase5-dev
```

O Makefile agora resolve os paths via `pkg-config Qt5Core Qt5Gui` (item 10). Nenhuma outra
parte do build principal usa Qt.

Próximo passo: `04_bibliotecas_sem_pacote_apt.md`.
