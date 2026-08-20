# Changelog da migração — alterações de código/Makefile

Tudo abaixo está na branch `ubuntu26`, commitado em `e666f3127` ("migração geral")
e ainda **não publicado**. Ver `CHECKLIST_COMMIT.md` para o levantamento do que
entrou e o que ficou de fora.

Ambiente: Ubuntu 26.04 LTS "resolute", gcc/g++ 15.2.0, CMake 4.2.3, GNU Make, Python 3.14.
Todas as alterações abaixo foram feitas rodando `make` de verdade e recompilando o módulo
afetado antes de seguir.

## Sessão 1 — 2026-08-18

### 1. Removido o prefixo hardcoded `/usr/local/carmen_boost`

28 `Makefile`s (módulos em `src/`) tinham, no meio das linhas de flags:

```make
IFLAGS += ... -I/usr/local/carmen_boost/include
LFLAGS += ... -L/usr/local/carmen_boost/lib
```

Esse é o Boost 1.61 que o tutorial de 16.04/20.04 mandava compilar à mão e instalar num
prefixo próprio. Numa instalação nova do Ubuntu 26.04 esse diretório não existe, e o build
"pede um Boost que não está lá".

**Correção:** as duas flags foram removidas (não substituídas por outro path). Sem elas, o
compilador cai nos diretórios padrão (`/usr/include`, `/usr/lib/x86_64-linux-gnu`), que é
exatamente onde `apt install libboost-all-dev` instala — ver `install/02_boost.md`.

**Não mexido (de propósito):** `src/ros1_sc_lio_sam/src/carmen_ipc_bridge/CMakeLists.txt` e
`src/ros1-lt-mapper/ros_ipc_carmen/src/carmen_ipc_bridge_noetic/CMakeLists.txt`, que têm o
mesmo hardcode. São builds `catkin` separados (ROS), fora do `make` do `src/` — ver
`install/05_pendencias_conhecidas.md`.

### 2. `-lboost_thread-mt` → `-lboost_thread` (94 Makefiles)

O sufixo `-mt` era a convenção de nome do Boost pré-1.5x para as libs thread-safe. O Boost do
apt (1.90) só gera `libboost_thread.so`, sem sufixo — daí `cannot find -lboost_thread-mt`.
150 ocorrências, em 94 arquivos.

Ficou **de fora** de propósito: `src/semantic_segmentation/caffe-segnet/Makefile`, onde
`boost_thread-mt` está dentro do ramo de macOS do Makefile do Caffe (código vendorizado, não
faz parte de nenhum pacote do `src/Makefile`).

### 3. Removida a flag `-lboost_signals` + porte do `libtf` para `Boost.Signals2`

`Boost.Signals` (v1) foi removido do Boost há vários releases; não existe pacote
`libboost-signals-dev` no Ubuntu 26.04. A flag foi removida (146 ocorrências).

O único uso real da API (não só a flag) está em `sharedlib/libtf/src/{signalslib.h,tf.h,tf.cpp}`
— a lib `tf` (transformações de coordenadas), de que praticamente todo módulo depende via
`-ltf`. Portado para `Boost.Signals2` (header-only, ainda existe, não precisa de `-l`):

- `signalslib.h`: `#include <boost/signal.hpp>` → `#include <boost/signals2.hpp>`;
  `namespace signalslib = signals;` → `namespace signals = signals2; namespace signalslib = signals2;`
- `tf.h`: `#include <boost/signals.hpp>` → `#include <boost/signals2.hpp>`;
  `typedef boost::signal<void(void)> TransformsChangedSignal;` →
  `typedef boost::signals::signal<void(void)> TransformsChangedSignal;`
- `tf.cpp` não precisou de mudança (já usava `boost::signals::connection`, que passa a
  resolver para `signals2` pelo alias).

É o mesmo diff que um fork deste código-base já tinha no `libtf2`.

### 4. Removida a flag `-lboost_system` (117 Makefiles)

`Boost.System` é header-only há vários releases e o pacote `libboost-all-dev` do 26.04 não
gera mais a `.so` de compatibilidade — `cannot find -lboost_system`. Como a biblioteca não
existe mais como artefato linkável, remover a flag é seguro (funciona também em Ubuntu antigo,
onde o `.so` de compat existia mas não era necessário).

### 5. `pkg-config opencv` → fallback `opencv4` (201 Makefiles)

Distros modernas só trazem `opencv4.pc`; o nome antigo `opencv.pc` não existe mais. Todas as
chamadas viraram, por exemplo:

```make
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`
```

Cobertas as 5 variantes de escrita que apareciam no repo (`--libs opencv`, `--cflags opencv`,
`opencv --libs`, `opencv --cflags`, `--libs --cflags opencv`). O fallback mantém o build
funcionando em máquinas antigas com só o `opencv.pc`.

### 6. `src/Makefile.conf`: auto-detecção de PCL, VTK, SuiteSparse (cs.h) e Bullet

Dezenas de Makefiles fixam versão/paths antigos: `-I/usr/local/include/pcl-1.8`,
`-I/usr/local/include/vtk-5*`, `-I/usr/local/include/EXTERNAL/csparse/`,
`-I/usr/local/include/bullet/`. Todos são resquícios das instalações manuais do tutorial de
16.04/20.04; no 26.04 esses pacotes vêm do apt e ficam em `/usr/include/...`.

Como `-I` para diretório inexistente é inofensivo (o gcc ignora), a correção foi **aditiva e
num único lugar** (`src/Makefile.conf`, incluído por todo módulo), em vez de editar cada
Makefile:

```make
IFLAGS += $(addprefix -I,$(wildcard /usr/local/include/pcl-* /usr/include/pcl-*))
IFLAGS += $(addprefix -I,$(wildcard /usr/local/include/vtk-* /usr/include/vtk-*))
IFLAGS += -I/usr/include/suitesparse
IFLAGS += -I/usr/include/bullet
VTK_LIB_SUFFIX := $(patsubst libvtkCommonCore-%.so,%,$(notdir $(firstword $(wildcard ...))))
```

- **PCL**: o `libpcl-dev` do 26.04 instala em `/usr/include/pcl-1.15`.
- **cs.h / g2o**: o g2o em si não precisa de `-I` (o código inclui `"g2o/core/..."` e o pacote
  instala em `/usr/include/g2o`, path padrão). O que precisa é o `cs.h`: o header
  `g2o/solvers/csparse/csparse_extension.h` faz `#include <cs.h>` sem prefixo, e no apt o
  `cs.h` vem do `libsuitesparse-dev`, em `/usr/include/suitesparse`.
- **Bullet**: o `libtf` inclui `LinearMath/*.h`; o `libbullet-dev` instala em
  `/usr/include/bullet`.
- **VTK**: o sufixo de versão faz parte do *nome da lib* (`-lvtkRenderingCore-6.3`), e o VTK
  não publica `.pc` nem alias sem versão — por isso um `-I`/`-L` extra não resolve.
  `VTK_LIB_SUFFIX` detecta a versão realmente instalada (9.5 aqui) para os Makefiles que
  precisarem.

### 7. `-I $(PCL_INC)` com espaço → `$(addprefix -I,$(PCL_INC))` (38 Makefiles)

Padrão perigoso encontrado em 38 Makefiles:

```make
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)
```

`PCL_INC`/`VTK_INC` são definidos como `$(wildcard /usr/local/include/pcl-*)` — path antigo,
que **não existe** no 26.04. A variável avalia vazia e `-I $(PCL_INC)` vira literalmente um
`-I` sozinho na linha de comando. Um `-I` sem argumento faz o gcc consumir o **próximo token**
como se fosse o valor do `-I` — inclusive o `-c`, que some do comando. O gcc então tenta
*linkar* um `.cpp` sem `main()`, e o erro que aparece é `undefined reference to 'main'` **ao
compilar um .o** — completamente desconectado da causa.

Trocado por `$(addprefix -I,$(PCL_INC))`, que produz *nada* quando a variável é vazia (e
continua idêntico quando não é). Com o item 6 resolvendo a detecção de verdade, essas variáveis
viraram redundantes, mas ficaram por segurança.

### 8. `sensors/web_cam` e `sensors/ultrasonic` adicionados ao `PACKAGES`

Erro logo no começo do build, em `src/global`:

```
include/carmen/writelog.h:34: fatal error: carmen/web_cam_interface.h: No such file or directory
include/carmen/writelog.h:35: fatal error: carmen/ultrasonic_filter_messages.h: No such file or directory
```

`src/logger/writelog.h` (header público, exportado para `include/carmen/`) inclui headers de
dois módulos de `src/sensors/` que **não estavam em nenhuma lista do `src/Makefile`** — ou
seja, ninguém exportava esses headers para `include/carmen/`. Numa máquina antiga isso passava
despercebido porque o `include/carmen/` já tinha os arquivos de builds anteriores; numa árvore
limpa o build não fecha.

Adicionados os dois ao `PACKAGES` (é onde um fork deste mesmo código já
lista `sensors`). Não adicionei o diretório `sensors` inteiro para não arrastar
`camera`/`minoru`/`ouster_sdk_driver_5-2`, que dependem de SDKs proprietários.

### 9. `carmen_stdio`: `gzFile *` (ponteiro duplo) e um `gzflush` no ponteiro errado

`src/global/carmen_stdio.h` declarava `gzFile *comp_fp;` — mas `gzFile` do zlib **já é um
ponteiro** (`typedef struct gzFile_s *gzFile`), então isso era um ponteiro-para-ponteiro
passado para todas as funções `gz*`. Até o GCC 13 era aviso
(`-Wincompatible-pointer-types`); no GCC 15 é erro. Trocado para `gzFile comp_fp;` (e o cast
`(gzFile)` que existia em `carmen_fgetc` ficou desnecessário).

Isso descobriu um **bug real de sempre** logo ao lado: `carmen_fflush()` chamava
`gzflush(fp->fp, Z_FINISH)` — passando o `FILE *` em vez do `gzFile`. Ou seja, dar flush num
log comprimido escrevia lixo/corrompia o handle. Corrigido para `gzflush(fp->comp_fp, ...)`.

### 10. `omp.h` dentro de `extern "C"` — "template with C linkage"

`src/localize_ackerman/localize_ackerman_core.h` faz `#include <omp.h>`, e esse header é
puxado pelo `carmen.h`, que envolve tudo num `extern "C" { }`. O `omp.h` do GCC 15 declara
**templates C++** (alocadores do OpenMP 5), e template com linkage C é erro:

```
/usr/lib/gcc/x86_64-linux-gnu/15/include/omp.h:459:1: error: template with C linkage
```

Corrigido envolvendo só esse include em `extern "C++" { }` (o `omp.h` tem o seu próprio
`extern "C"` interno para as funções C, então elas continuam com linkage C).

### 11. `-std=c++11` → `c++14` (Boost/tf) e `c++17` (PCL)

Dois requisitos novos, resolvidos por varredura:

- **PCL 1.15 exige C++17** — o próprio `pcl_config.h` emite `#error C++ standard too low (PCL
  requires 201703L or above)`. 44 Makefiles que usam PCL e estavam em `c++11`/`c++14`/`c++0x`
  foram para `-std=c++17`.
- **Boost 1.90 exige C++14** — o `libtf` inclui `boost/math`, que emite
  `#warning "Boost.Math requires C++14"` seguido de erro real (`'is_final' has not been
  declared in 'std'`). Importante: o header é reprocessado no `-std` de **quem inclui**, não no
  do `libtf`. Por isso subi de uma vez os 38 Makefiles que linkam `-ltf` e ainda estavam em
  `c++11`/`c++0x`, além do próprio `sharedlib/libtf/src/Makefile`.

### 12. `libtf`: `pt::microseconds(nsec/1000.0)` não compila mais no Boost 1.90

`tf_impl_time.h` e `tf_impl_duration.h` têm um fallback (usado quando
`BOOST_DATE_TIME_HAS_NANOSECONDS` não está definida, que é o caso aqui) fazendo
`pt::microseconds(nsec/1000.0)`. O `/1000.0` força divisão em ponto flutuante, e o construtor
de `boost::posix_time::microseconds` só aceita tipo integral (`enable_if<is_integral<T>>`).
Trocado por `/1000` (divisão inteira — que é o que faz sentido: truncar nanossegundos para
microssegundos).

### 13. `src/global`: Qt4 → Qt5 via `pkg-config`

`global_graphics_qt.cpp` não compilava: `QSocketNotifier: No such file or directory`. O
Makefile fixava os paths do Qt4 (`/usr/include/qt4/QtCore`, `/usr/lib64/qt4/mkspecs/...`) e
linkava `-lQtGui -lQtCore` — nomes/caminhos que não existem mais (Qt4 saiu do Ubuntu há
tempos). O arquivo só usa `QObject`/`QSocketNotifier`, idênticos no Qt5.

Trocado por `pkg-config --cflags/--libs Qt5Core Qt5Gui` com fallback para os nomes antigos, e
adicionado `-fPIC` (exigido pelos headers do Qt5). Pacote necessário: `qtbase5-dev`.

### 14. 38 módulos linkavam OpenCV sem nunca pedir os `-I` dele

`camera_drivers` falhou com `opencv2/highgui/highgui.hpp: No such file or directory` mesmo com
o `pkg-config --libs opencv4` no `LFLAGS`. Causa: o módulo nunca chamava `--cflags`. No Ubuntu
20.04 isso não dava erro porque os headers ficavam direto em `/usr/include/opencv2` (path
padrão do compilador); o pacote do 26.04 organiza tudo dentro de `/usr/include/opencv4`, que
exige `-I` explícito.

Varri o repo por Makefiles que linkam OpenCV, **não** pedem `--cflags` e cujo próprio código
inclui headers do OpenCV: 38 módulos. Todos ganharam a linha
`` IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv` ``.

### 15. `camera_drivers`: `initUndistortRectifyMap` saiu do `imgproc`

No OpenCV 4 as funções de calibração (`initUndistortRectifyMap`,
`getOptimalNewCameraMatrix`, ...) ficam só no módulo `calib3d`; antes chegavam por inclusão
transitiva. Adicionado `#include <opencv2/calib3d.hpp>` em
`camera_drivers_process_image.hpp`.

### 16. `#if CV_MAJOR_VERSION == 3` → `>= 3` (25 arquivos)

Padrão repetido em `mapper*`, `road_mapper`, `rddf_graph`, `traffic_light`, `v_disparity`,
`viewer_3D/texture_loader`, `download_map`, `neural_object_detector_point_cloud` e outros: um
`#if` que escolhe os headers modernos do OpenCV **só** quando a versão é exatamente 3, caindo
no `#else` (com `opencv/cv.h`/`opencv/highgui.h`, que não existem mais) para qualquer outra —
inclusive a 4, que é a instalada. Trocado `== 3` por `>= 3` nos 25 arquivos.

### 17. `sensors/web_cam`: headers da API C do OpenCV

`web_cam_main.c`/`web_cam_test.c` incluíam `<opencv/cv.h>`/`<opencv/highgui.h>` sem nenhum
`#if` de versão. A API C que eles usam (`IplImage`, `cvQueryFrame`, `cvShowImage`,
`cvNamedWindow`, ...) continua existindo no OpenCV 4, só mudou de arquivo — trocado por
`opencv2/core/core_c.h`, `opencv2/imgproc/imgproc_c.h`, `opencv2/highgui/highgui_c.h`.
Módulo compila e linka.

### 18. `maptools/map_graphics`: ponteiros de função estilo K&R

`GtkMapViewer` (struct usada por toda GUI de mapas) declarava seus 5 campos de callback como
`void (*campo)();` — a forma pré-ANSI de "argumentos não especificados". Até o GCC 13 atribuir
ou chamar isso com qualquer assinatura era só aviso; no GCC 14+ é erro
(`too many arguments to function 'map_view->user_draw_routine'; expected 0, have 1`).

Corrigido dando nome ao struct (`struct carmen_gtk_map_viewer`, antes anônimo) e tipando os 5
campos com a assinatura real — que já existia, mais abaixo no mesmo arquivo, nos `typedef`s
`carmen_graphics_mapview_*_t` usados para esses mesmos campos.

Isso expôs mais 4 erros do mesmo tipo em `map_graphics.c`: os handlers de sinal do GTK
(`motion_event`, `keyboard_press_event`, `button_release_event`, `button_press_event`)
passavam `GdkEventMotion*`/`GdkEventKey*`/`GdkEventButton*` onde a assinatura pede `GdkEvent*`
— sempre foi assim (é o idioma do GDK: todos são membros da union `GdkEvent`), só que sem
prototype o compilador não conferia. Adicionado o cast `(GdkEvent *)` explícito nas 4 chamadas.

### 19. `mapeditor`: `cvLoadImage` removida do binário do OpenCV 4

`map_editor_menus.c` usava `#if CV_VERSION_MAJOR == 3` (variante de escrita que a varredura do
item 16 não pegou — lá era `CV_MAJOR_VERSION`) e, no ramo do 3, incluía
`opencv2/imgcodecs/imgcodecs_c.h`, que no OpenCV 4 nem existe mais. Além do header, a função
`cvLoadImage()` foi **removida do binário** — não basta trocar include.

Adicionado um ramo `#elif CV_VERSION_MAJOR >= 4` com `opencv2/imgcodecs.hpp` e, em
`bmp_open()`, o porte para o loader C++ com ponte para `IplImage`:

```c
cv::Mat bmpfile_mat = cv::imread(filename, cv::IMREAD_GRAYSCALE);
IplImage bmpfile_ipl = cvIplImage(bmpfile_mat);
IplImage* bmpfile = &bmpfile_ipl;
```

Funciona apesar do arquivo ser `.c` porque o módulo já compila com `CC = g++` (e
`-fpermissive`) no próprio Makefile.

### 20. Constantes legadas do OpenCV (`CV_IMWRITE_*`, `CV_LOAD_IMAGE_*`) — 18 arquivos

`CV_IMWRITE_PNG_COMPRESSION`, `CV_IMWRITE_JPEG_QUALITY`, `CV_LOAD_IMAGE_COLOR`,
`CV_LOAD_IMAGE_GRAYSCALE`, `CV_LOAD_IMAGE_ANYCOLOR`, `CV_LOAD_IMAGE_UNCHANGED`: essas
constantes viviam no `imgcodecs_c.h`, que no OpenCV 4 só existe para dar `#error`. Trocadas
pelos nomes modernos (`cv::IMWRITE_*` / `cv::IMREAD_*`), que já existem desde o OpenCV 3 —
então o código continua compilando nas duas versões.

Feito nos 18 arquivos que estão em módulos realmente construídos pelo `src/Makefile`
(`logger`, `map_server`, `viewer_3D`, `navigator_gui2`, `download_map`, `mapeditor`,
`utilities/convert_log_images`, `stereo`, `road_finding`, `localize_neural`, `tracker_opentld`).
Arquivos de módulos fora do build (e código de terceiros vendorizado, como `sharedlib/darknet*`
e `caffe-segnet`) **não** foram tocados — ver `install/05_pendencias_conhecidas.md`.

⚠️ Importante: nem toda constante `CV_*` sumiu. `CV_RGB2BGR`, `CV_FONT_*`, `CV_FILLED`,
`CV_AA`, `CV_EVENT_*`, `CV_WINDOW_*` continuam existindo no OpenCV 4 — só que nos headers
`*_c.h` (`opencv2/imgproc/imgproc_c.h`, `opencv2/highgui/highgui_c.h`). Para essas, a correção
é o `#include` certo, não renomear.

### 21. `#include <opencv/...>` — 89 arquivos migrados para `opencv2/...`

Os headers da era OpenCV 1/2 (`opencv/cv.h`, `opencv/highgui.h`, `opencv/cxcore.h`,
`opencv/cv.hpp`, `opencv/ml.h`) não existem mais. A migração **não é uma troca única**: depende
do que o arquivo realmente usa. Classifiquei arquivo por arquivo (por grep de uso real de API
C: `IplImage`, `Cv*`, `cv*()`, `CV_FONT_*`, `CV_RGB()`, `CV_FILLED`, ...) e apliquei:

| Uso real do arquivo | `opencv/cv.h` vira | `opencv/highgui.h` vira |
|---|---|---|
| API C (`IplImage`, `cvPoint`, ...) | `opencv2/core/core_c.h` + `opencv2/imgproc/imgproc_c.h` | `opencv2/highgui/highgui_c.h` |
| Só C++ (`cv::Mat` e afins) | `opencv2/core.hpp` | `opencv2/highgui.hpp` |

Resultado: 58 arquivos foram para os headers `*_c.h`, 43 para os headers C++, num total de 89
alterados. Ficaram de fora (de propósito) 12 arquivos que **já tinham** um `#if
CV_MAJOR_VERSION` protegendo o include — esses foram tratados individualmente (itens 19, 22 e
23), porque neles o `#else` legado deve continuar existindo.

### 22. `viewer_3D`: `cvLoadImage`/`cvConvertImage` removidas do binário do OpenCV 4

Diferente das constantes (item 20) e dos includes (item 21), aqui as **funções** deixaram de
existir — não basta trocar header:

- `texture_loader.cpp`:
  - o `#if` de versão incluía `opencv2/imgcodecs/imgcodecs_c.h`, que dá `#error` no OpenCV 4;
    adicionado um ramo `#elif CV_MAJOR_VERSION >= 4` com os headers certos;
  - `rotate_map()` usava `cvConvertImage(..., CV_CVTIMG_FLIP)`, removida sem equivalente na
    API C. Trocado por `cv::flip(mat, mat, 0)` sobre uma view (`cv::cvarrToMat`, sem cópia) —
    o flip é in-place e não precisa sobreviver ao escopo;
  - `create_texture()` usava `cvLoadImage`. Trocado por `cv::imread` + `cvIplImage` +
    **`cvCloneImage`** — aqui o `IplImage` fica guardado numa global, então uma view sobre a
    `cv::Mat` local morreria.
- `viewer_3D.cpp` (`find_map_from_data`): mesmo porte com `cvCloneImage`, porque a função
  *retorna* o ponteiro para o chamador, que depois dá `cvReleaseImage`.
- `3D_map_view.cpp`: `#include <opencv/cv.h>` (usa `cvPoint`/`cvScalar`/`CV_FONT_*`) →
  headers `*_c.h`.

### 23. `viewer_3D`: `std::beta` do C++17 colidindo com uma variável global `beta`

`viewer_3D.cpp` declara `static double beta[MAX_NUM_TRAILERS]` e usa `using namespace std`. Em
C++17 o `<cmath>` passou a trazer as *mathematical special functions*, entre elas `std::beta` —
o nome ficou visível sem qualificação e deu `error: reference to 'beta' is ambiguous` em 10
pontos do arquivo. Como o módulo agora compila em C++17 (item 11), isso apareceu.

Renomeada a global para `g_beta` (convenção que o próprio repo já usa). Confirmei antes que ela
não é `extern` em nenhum outro arquivo, e que as demais ocorrências de `beta` no arquivo são
variáveis locais (que sombreiam a global e nunca deram erro).

### 24. `bumblebee_basic`: `-I` do OpenCV em `CFLAGS` e o SDK FlyCapture

Dois problemas no mesmo módulo:

1. O `pkg-config --cflags opencv4` estava em **`CFLAGS`**, e a regra `%.o: %.cpp` do
   `Makefile.rules` usa `$(CXXFLAGS) $(IFLAGS)` — nunca `$(CFLAGS)`. Como o módulo só tem
   fontes `.cpp`, o `-I` jamais chegava ao compilador. Movido para `IFLAGS`.
   `bumblebee_basic_view.cpp` também tinha o `opencv/cv.h` morto (item 21), trocado pelos
   headers `*_c.h`.
2. O driver `bumblebee_basic` linka `-lflycapture -ltriclops -lpnmutils` — SDK proprietário
   da PointGrey/FLIR, sem pacote apt (ver `install/04`). Sem ele, `cannot find -lflycapture`
   derrubava o build inteiro. O Makefile já tinha um corte parecido para `aarch64`; troquei
   por uma **detecção do SDK** (`HAS_FLYCAPTURE` via `wildcard`): quando a lib não está
   instalada, só o binário do driver deixa de ser construído — a interface (`libbumblebee_
   basic_interface`, usada por vários outros módulos), o `bumblebee_basic_simulator` e o
   `bumblebee_basic_view` continuam sendo compilados. Instalar o SDK reativa o driver sem
   editar nada.

## Sessão 1 (continuação) — fase de binários (`phase2`)

Marco: com os itens 1–24 a **`phase1` inteira (bibliotecas de todos os módulos) fechou**, e o
build passou para a `phase2`, onde os binários são linkados e copiados para `bin/`. Os erros
daqui em diante são de outra natureza — a maioria é de **link**, não de compilação.

### 25. `python2.7` e versões fixas de Python nos Makefiles

O Ubuntu 26.04 traz Python 3.14 e não tem mais Python 2. Dois padrões foram corrigidos:

- **`-I/usr/include/python3.5|3.6|3.8` e `python3.X-config --libs`** (29 Makefiles): trocados
  por `pkg-config --cflags python3` / `pkg-config --libs python3-embed` com fallback para
  `python3-config --embed --libs`. O `--embed` é obrigatório desde o Python 3.8 para quem
  linka o interpretador dentro de um binário C/C++.
- **`-lpython2.7` / `-I/usr/include/python2.7`** (`lane_detector`, `yolo_save_prediction`,
  `neural_object_detector*`, `voice_interface/libvoice_google`, `parking_assistant`,
  `sharedlib/Deeplab`, `sharedlib/salsanet`, `sharedlib/libsqueeze_seg_v2` e outros):
  trocados pelos equivalentes de Python 3.

Além disso, `src/Makefile.conf` passou a acrescentar **globalmente** os `-I` do Python e do
numpy (detectados via `pkg-config` e via `python3 -c "import numpy; print(numpy.get_include())"`),
porque vários módulos incluem `<Python.h>`/`<numpy/arrayobject.h>` **transitivamente** (por
`carmen/voice_interface_interface.h`) sem nunca terem pedido esses `-I` no próprio Makefile.

### 26. `proccontrol_gui`: Qt3Support + `qmake-qt4` — desativado, não portado

O `proccontrol_gui` é construído por fora do sistema de build do CARMEN, chamando
`qmake-qt4`, e o `proccontrol.pro` pede `QT += qt3support`. Não é só o `qmake` que sumiu: o
código usa **classes de compatibilidade Qt3** (`Q3ButtonGroup`, `Q3PopupMenu`, `Q3TextView`,
`Q3HBoxLayout`, `Q3VBoxLayout`), removidas no Qt5. Portar isso é um trabalho de GUI de
verdade (Q3PopupMenu→QMenu, Q3TextView→QTextBrowser, Q3ButtonGroup→QButtonGroup+QGroupBox,
Q3*Layout→Q*Layout), com ~600 linhas para revisar e sem como validar sem rodar a interface.

Como o binário é **opcional** (o `proccontrol`, que de fato executa os módulos, não depende
dele), o alvo passou a ser condicionado à existência do `qmake-qt4`
(`HAS_QMAKE_QT4 := $(shell which qmake-qt4 ...)`): sem ele, o resto do módulo compila
normalmente; se alguém instalar/portar, o alvo volta sozinho. Detalhes e roteiro de porte em
`install/05_pendencias_conhecidas.md`.

### 27. Módulos que dependem de SDK/lib externa passaram a detectá-la em vez de quebrar o build

O `make` do topo não tolera falha (o `RECURSE` do `src/Makefile` é um `for` de shell com
`exit -1`): um módulo que não linka derruba o build inteiro. Vários módulos dependem de coisas
que não têm pacote apt e nem todo mundo instala. Em vez de removê-los da lista `PACKAGES`
(o que esconderia o módulo), cada um passou a **detectar a dependência**:

| Módulo | Dependência | Sem ela |
|---|---|---|
| `bumblebee_basic` | FlyCapture2/Triclops (PointGrey/FLIR) | Sai só o driver; interface, simulator e view continuam |
| `traffic_light` | `libwnn` (repo do LCAD) + CUDA/Caffe | Sai só `libtraffic_light_interface` — que é o que o `rddf` precisa |
| `tracker` | MAE (`$MAEHOME`, compilador `netcomp`) | Sai só `libtracker_interface` |
| `sensors/kinect` | `libfreenect` (existe no apt, não vem instalada) | Sai só `libkinect_interface` |
| `proccontrol_gui` | `qmake-qt4` + Qt3Support | Não é construído (item 26) |

Todos são reversíveis sem editar nada: basta instalar a dependência e rodar `make` de novo.

### 28. `sensors/camera` e `traffic_light` entraram no `PACKAGES`

Mesmo caso do item 8: `pi_camera` inclui `<carmen/camera_messages.h>` e `rddf` inclui
`<carmen/traffic_light_interface.h>` + linka `-ltraffic_light_interface`, mas nenhum dos dois
módulos estava em lista alguma do `src/Makefile` — ou seja, numa árvore limpa esses headers e
libs nunca eram gerados. Ambos foram adicionados ao `PACKAGES` (o `traffic_light` com o
gating do item 27).

### 29. Símbolos duplicados: o fim do `-fcommon` (GCC 10+)

Até o GCC 9, o default `-fcommon` fundia num símbolo só as *definições tentativas* de uma
mesma variável global em vários arquivos-objeto. Do GCC 10 em diante o default é
`-fno-common` e isso vira `multiple definition` no link. Apareceu em 4 lugares:

- **`logger/print_binary_log.cpp`**: redefinia `MSG_INSTANCE current_msgRef`, que já é
  definida em `global/ipc_wrapper.c` (e declarada `extern` em `ipc_wrapper.h`). A variável
  nem era usada no arquivo — definição removida.
- **`sensors/laser_new/carmen_laser_device_init.c`**: redefinia
  `carmen_laser_configurations`/`carmen_laser_configurations_num`, já definidas em
  `carmen_laser_device.c` e declaradas `extern` no header. Viraram `extern`.
- **`sharedlib/libstereo/qx_csbp.h`**: 15 variáveis globais eram **definidas dentro do
  header**, incluído por 4 arquivos. Viraram `extern` no header, com a definição única movida
  para `qx_csbp_OMP.c`.
- **`sharedlib/libstereovgram/vg_ram.c`**: `DisparityMap disp;` — global com nome genérico que
  colidia com o `disp` (outro símbolo, sem relação) da `libstereo`. Como só é usado dentro do
  arquivo, virou `static`.

### 30. `pantilt-test`: `struct termio` (System V) removida da glibc

`src/pantilt/pantilt-test.c` usava `struct termio` + `ioctl(TCGETA/TCSETA)`, a API de terminal
do System V, que não existe mais na glibc do 26.04. O próprio arquivo já tinha o caminho POSIX
equivalente (`struct termios` + `tcgetattr`/`tcsetattr`) num `#ifdef __APPLE__` — os dois ramos
viraram um só, mantendo o comportamento do lado Linux (aplica o modo `raw`, restaura o
`cooked` na saída).

### 31. FFmpeg 8: `AVCodec *` → `const AVCodec *`

`camera_drivers_access_image_functions.cpp` — `av_find_best_stream()` e
`avcodec_find_decoder()` do FFmpeg atual devolvem/esperam `const AVCodec *`. Como os ponteiros
só são lidos, bastou declará-los `const`.

### 32. `car_panel_gui`: `-lQtCore` → Qt5

Consequência do item 13 (o `global_graphics_qt` agora é compilado contra Qt5): o
`car_panel_gui`, que linka contra ele, também precisava trocar o `-lQtCore` (nome do Qt4) pelo
do Qt5, via `pkg-config --libs Qt5Core` com fallback para o nome antigo.

### 33. `navigator_gui2`: faltava `-lX11` explícito

`gtk_gui.cpp` chama `XRecolorCursor()` direto (pegando `Display*`/`Cursor` do GDK). O
`pkg-config --libs gtk+-2.0` não devolve mais `-lX11`, e o `ld` do 26.04 não aceita resolver
símbolo através de uma DSO indireta (`DSO missing from command line`). Adicionado `-lX11`.

### 34. Mais API C do OpenCV nos módulos de visão

Continuação dos itens 20–22, agora nos módulos que só são linkados na `phase2`:

- **`src/global/opencv_c_compat.h` (arquivo novo)** — em vez de repetir o mesmo porte em cada
  arquivo, criei um header de compatibilidade, exportado como `<carmen/opencv_c_compat.h>`,
  que reimplementa `cvLoadImage()`/`cvSaveImage()` em cima de `cv::imread`/`cv::imwrite` e
  redefine `CV_RGB()` para devolver `CvScalar`. Em OpenCV ≤ 3 ele não faz nada. Usado por
  `logger`, `log_filter`, `stereo`, `stereo_mapping`, `map_server`, `road_finding`,
  `localize_neural`, `tracker_opentld`, `visual_tracker` e `utilities/*`.
- **`CV_RGB` redefinido localmente** em 13 arquivos que desenham pela API C (o `CV_RGB` que
  sobra no OpenCV 4 é o do `imgproc.hpp`, que devolve `cv::Scalar` e não converte para
  `CvScalar`).
- **Headers `*_c.h` faltando** em 63 arquivos que usam `IplImage`/`cvScalar`/`CV_FONT_*`/
  `cvDestroyAllWindows` etc. — esses símbolos continuam existindo no OpenCV 4, só não chegam
  mais por inclusão transitiva.
- **`tracker_opentld`**: porte maior — `CvCapture` (saiu para `videoio_c.h`),
  `cvCalcOpticalFlowPyrLK` (**removida**, portada para `cv::calcOpticalFlowPyrLK` num
  adaptador que mantém a assinatura em `CvPoint2D32f`), conversão implícita
  `cv::Mat → IplImage` (agora `cvIplImage()`), `cv::Rect`↔`CvRect` e `cv::Point`↔`CvPoint`
  (sem conversão implícita no OpenCV 4).
- **`stereo_mapping`**: `CvKalman` (filtro de Kalman da API C) foi **removido por completo** do
  OpenCV 4. O `typedef` que o usava não era referenciado em lugar nenhum do repositório (o
  código que roda o filtro já usa `cv::KalmanFilter`), então foi removido em vez de portado.

### 35. `tracker_opentld/3rdparty/libconfig`: parser gerado por bison/flex vs. C23

O `libconfig` vendorizado tem código gerado com protótipos K&R (`int f()` = "argumentos não
especificados"). O GCC 15 usa **C23** por padrão, onde `f()` passou a significar `f(void)` — e
chamar com argumentos virou erro. Além disso a parte C++ da lib usa *dynamic exception
specifications* (`throw(ParseException)`), removidas no C++17 (o default do GCC 15).

Como é código de terceiros congelado, o diretório passou a compilar com `-std=gnu17` (C) e
`-std=gnu++14` (C++), em vez de regenerar o parser ou reescrever a lib.

### 36. Mais handlers K&R passados para o IPC

Mesma classe do item 18, agora em ponteiros de função de mensagem/timer:

- `motion_planner_main.c`: `go_handler()`/`stop_handler()` são passados direto para
  `carmen_subscribe_message()`, que espera `carmen_handler_t` = `void (*)(void *)`.
- `navigator_astar/startgoal_simulator.c` e `path_planner/startgoal_simulator.c`:
  `periodic_publish_globalpos()` é registrada como `TIMER_HANDLER_TYPE`, que exige três
  argumentos.

Em todos, a correção foi declarar a assinatura real com `__attribute__((unused))` nos
parâmetros não usados — mesmo comportamento, só satisfazendo o type-checking.

### 37. `lane_detector`: `abs(unsigned int)` ambíguo em C++17

`calculate_the_distance_point_to_the_line()` chamava `abs()` sobre uma expressão `unsigned int`
(os campos da struct `lines` são unsigned). Com o `<cmath>` do C++17 trazendo as sobrecargas de
`std::abs` para ponto flutuante, a chamada ficou ambígua. Convertido para `fabs((double) ...)`,
que é o que a conta quer dizer — o valor não muda, já que o operando era não-negativo.

### 38. `OpenJAUS/ojTorc`: `GEAR_NUMBER` era uma *variável* declarada no header

Mesma classe do item 29 (`-fno-common` do GCC 10+), mas com uma causa mais sutil:

```c
enum
{
        GEAR_1 = 3,
        ...
} GEAR_NUMBER;      /* <- isto declara uma VARIÁVEL, não um tipo */
```

Faltou o `typedef`. Como estava, `include/torc.h` declarava uma variável global chamada
`GEAR_NUMBER` em toda unidade de tradução que incluísse o header — três delas (`torc.c`,
`torcComm.c`, `torcInterface.c`). Com `-fcommon` (padrão até o GCC 9) o linker fundia essas
definições tentativas num símbolo só; com `-fno-common` (padrão do GCC 10+) virou
`multiple definition of 'GEAR_NUMBER'` ao linkar `lib/libojTorc.so` e `bin/ojTorc`.

A variável nunca foi lida nem escrita em lugar nenhum do repositório (`grep -rn GEAR_NUMBER`
só encontra esta linha) — o que o código sempre usou foram os enumeradores (`GEAR_1`,
`REVERSE_GEAR_1`, …). A intenção original era claramente nomear o tipo, como diz o comentário
logo acima do enum. Corrigido para `typedef enum { ... } GEAR_NUMBER;`, que transforma o nome
num tipo e elimina o símbolo duplicado sem mudar comportamento.

⚠️ O `Makefile` do OpenJAUS **não gera dependências de header** (`Makefile.depend`): depois de
editar `include/torc.h`, os `.o` em `ojTorc/Build/` não são recompilados e o mesmo erro de link
se repete. Foi preciso limpar antes de rebuildar:

```bash
cd $CARMEN_HOME/sharedlib/OpenJAUS/ojTorc
rm -f Build/*.o lib/libojTorc.so bin/ojTorc
```

### 39. `visual_car_tracking`: `CV_HAAR_SCALE_IMAGE` removida no OpenCV 4

`car_cascade.detectMultiScale(..., 0|CV_HAAR_SCALE_IMAGE, ...)`. Essa constante vem da API C
do detector Haar (`objdetect/objdetect_c.h`), que foi **removida** no OpenCV 4 — ao contrário
dos símbolos do item 34, aqui não adianta incluir o `*_c.h`. O código já usa a API C++
(`cv::CascadeClassifier`), cujo equivalente é `cv::CASCADE_SCALE_IMAGE` (mesmo valor, 2, e já
disponível pelo `objdetect.hpp` que o módulo inclui). Trocado.

### 40. `graphslam`: `Ptr` do PCL deixou de ser `boost::shared_ptr`

A partir do PCL 1.11 (aqui é 1.15) `pcl::PointCloud<T>::Ptr` é `std::shared_ptr<T>`, não
`boost::shared_ptr<T>` — e não há conversão entre os dois. O código construía o ponteiro
nomeando o tipo concreto antigo:

```cpp
source_pointcloud = boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
```

Trocado pelo próprio alias do PCL, que é o que o código sempre quis dizer e funciona nas duas
versões:

```cpp
source_pointcloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
```

11 ocorrências em `velodyne_util.cpp`, `multimap_matching.cpp`, `run_icp.cpp` e
`run_icp_for_loop_closure.cpp`.

O mesmo padrão existe em `visual_graphslam`, `voslam`, `slam_icp` e `deep_vgl`, que **não estão
na lista `PACKAGES`** do `src/Makefile` e portanto não são compilados — não foram alterados
para não mexer em código que não dá para verificar aqui. Está anotado em
`install/05_pendencias_conhecidas.md`.

### 41. `graphslam`: `Factory::registerType()` do g2o agora recebe `shared_ptr`

O g2o do 26.04 é o `libg2o-dev 0~20230806-5`. Nessa versão a fábrica de tipos guarda
`std::shared_ptr<AbstractHyperGraphElementCreator>` em vez de um ponteiro cru:

```cpp
// antes
factory->registerType("EDGE_GPS", new HyperGraphElementCreator<EdgeGPS>);
// agora
factory->registerType("EDGE_GPS", std::make_shared<HyperGraphElementCreator<EdgeGPS> >());
```

Corrigido em `graphslam.cpp` (2 registros: `EDGE_GPS` e `EDGE_GPS_NEW`) e em
`multimap_optimization.cpp` (1). O `graphslam_guarapari.cpp` tem o mesmo registro mas **não
está no `SOURCES` do Makefile do módulo** — não é compilado, e por isso não foi mexido.

### 42. `graphslam`: `Registration::setInputCloud()` virou privada no PCL

Em `pcl::Registration` (base de `GeneralizedIterativeClosestPoint`), `setInputCloud()` foi
renomeada para `setInputSource()`; a antiga continua existindo mas foi movida para a seção
`private`, então o erro é `is private within this context` — e não "não declarada".

Trocado `gicp.setInputCloud(...)` → `gicp.setInputSource(...)` em `run_icp_for_loop_closure.cpp`
(2 chamadas) e `multimap_matching.cpp` (1). A semântica é a mesma: a nuvem "de origem", que já
era o par de `setInputTarget()` logo abaixo em todas as chamadas.

⚠️ **Não confundir**: `setInputCloud()` de `VoxelGrid`, `KdTreeFLANN`,
`EuclideanClusterExtraction`, `PCA` e `MomentOfInertiaEstimation` continua sendo API pública e
correta — só a de `Registration` mudou. As chamadas em `grid.setInputCloud(...)` foram mantidas.

### 43. Runtime: `exit 127` no `proccontrol` — soname antigo e binário inexistente

Primeiro erro da fase de runtime. `./proccontrol argos/process-argos-navigate.ini` respawnava
em loop três módulos com `exited UNCLEANLY (code = 127)`: `proccontrol_gui`, `route_planner` e
`offroad_planner`. São **duas causas diferentes** com o mesmo código de saída:

**a) `proccontrol_gui` — o binário não existe.** É a pendência a): `proccontrol_gui.cpp` usa
Qt3Support (só existia no Qt4), nunca foi portado, então o `make` não o gera. Como é apenas a
GUI de supervisão e o `proccontrol` roda sem ela, foi desligado no `.ini`
(`requested_state` 1 → 0), com comentário explicando como religar:

```
 proc_control			support 		0		0			./proccontrol_gui
```

Só o `.ini` que o Miguel estava rodando foi alterado. **Existem ~1030 referências a
`proccontrol_gui` nos demais `process-*.ini`** de `bin/` — todos vão dar o mesmo 127 enquanto
o porte para Qt5 não for feito.

**b) `route_planner` e `offroad_planner` — soname que não existe mais.** Pediam
`libgsl.so.23` e `libpython3.8.so.1.0`; o 26.04 tem `libgsl.so.28` e `libpython3.14`. Quando o
loader não resolve uma `.so`, o processo morre com 127 — igual a "comando não encontrado", o
que despista. Esses módulos **não têm fonte no repositório**: são binários commitados,
compilados no 20.04. Resolvido com shims de soname em `lib/compat` — ver
`install/05_pendencias_conhecidas.md`, seção **i)**, que traz a validação por símbolo e as
ressalvas de ABI.

Depois das duas correções, `proccontrol` sobe com **zero** ocorrências de `code = 127` e
nenhum `exited UNCLEANLY`.

### 44. `proccontrol_gui`: reescrito em GTK3 — **resolve a pendência a)**

O `proccontrol_gui` era o último binário do build principal que não compilava. Não era falta
de comando: o `proccontrol.pro` pedia `QT += qt3support` e o código usava classes do módulo de
compatibilidade Qt3, que morreu junto com o Qt4.

Cheguei a fazer o porte direto para Qt5 (funcionou), mas **um fork deste código já tinha resolvido isso de
um jeito melhor**: lá o GUI foi *reescrito* em GTK3 (`proccontrol_gtk.c`), e os
`proccontrol_gui.{h,cpp}` do Qt ficaram como arquivos mortos. Foi o caminho adotado aqui
também, porque:

- **tira o Qt do build.** O CARMEN não precisa mais de `qtbase5-dev` nem de um passo de build
  à parte — sem `qmake`, sem `moc`, sem `.pro`. O `proccontrol_gui` virou um alvo normal do
  `Makefile` do módulo, como qualquer outro binário;
- **converge os dois repositórios.** O fork mantém a mesma
  estrutura de build; ter duas GUIs diferentes para a mesma ferramenta era dívida garantida;
- Qt5 é fim de linha; GTK3 é o que o resto das GUIs do CARMEN já usa (em GTK2), e
  `libgtk-3-dev` já é dependência do sistema.

**O layout e o comportamento continuam os do CARMEN**, não os do fork — o que foi
aproveitado de lá foi a estrutura do código (`GHashTable` de grupos e módulos, menu por botão
via `g_object_set_data`, cor por CSS, `GMainLoop`), não a aparência:

| | CARMEN (mantido) | fork (não adotado) |
|---|---|---|
| Menu do módulo | Start Program / Stop Program / Show Output / **No Output** | Start / Stop / Show Output / Hide Output |
| Menu do "All" | Start / Stop, via `carmen_proccontrol_set_group_state()` (uma mensagem de grupo) | Start All / Stop All + saída, iterando módulo a módulo |
| Barra global "Enable/Disable All Outputs" | não existe | existe |
| Cores | verde `rgb(60,192,34)`, vermelho `rgb(221,0,3)`, amarelo instável, texto preto | verde/vermelho puros, com variante translúcida |
| Assinatura da saída | sob demanda: assina ao ligar o primeiro "Show Output", cancela ao desligar o último | sempre assinada |
| Preferências | `user_preferences` (tamanho/posição da janela), mesmas chaves de antes | `-list_module` / `-show_output` |
| Linha de comando | `-show modulo1 modulo2 ...` | `-list_module` / `-show_output` |

A estrutura visual é a mesma de antes: grupos empilhados verticalmente, cada um num quadro com
o nome do grupo contendo **uma linha** de botões (o "All" estreito primeiro, depois um botão
por módulo, cada um com `nome\npid: N`), e a área de saída no rodapé em Courier 10, limitada a
500 linhas.

Detalhe de tradução do laço principal: o GUI Qt rodava
`app.processEvents(); carmen_ipc_sleep(0.02)`. Em GTK quem manda no laço é o `GMainLoop`, então
o dispatch do IPC entrou como um `g_timeout_add(20ms, ...)` chamando
`carmen_ipc_dispatch_nonblocking()` — mesma cadência, sem bloquear a interface.

⚠️ **Wayland.** O `proccontrol_gui` agora é GTK3 e, numa sessão Wayland, usa o backend Wayland —
ao contrário das outras GUIs do CARMEN (GTK2), que caem no XWayland. Consequência prática: ele
**não aparece** em ferramentas X11 como `xwininfo`/`import`. A janela existe normalmente; para
depurar com ferramentas X, rode com `GDK_BACKEND=x11`.

O `proccontrol.pro`, o `proccontrol_gui.cpp` e o `proccontrol_gui.h` foram **deixados como
estavam** — não são mais compilados por nada, como já acontece no fork. Removê-los é uma
decisão à parte, de limpeza.

---

## Sessão 2 — 2026-08-20

### 45. `viewer_3D`: desligado o vsync — era ele que fazia o `playback` engasgar

**Sintoma.** Com a stack de playback de pé (`bin/argos/process-argos-playback.ini`), o
`playback` engasgava: travava por ~1 s, e em seguida recuperava o atraso reproduzindo a ~2x.
Acontecia a cada poucos segundos, de forma irregular, e o mesmo padrão aparecia na stack de
simulação.

**Causa.** O desenho do `viewer_3D` roda **dentro do handler de timer do IPC** —
`carmen_ipc_addPeriodicTimer(1.0 / 40.0, draw_timer_handler, NULL)`, em `viewer_3D.cpp`. Logo,
todo tempo que o `glXSwapBuffers()` fica bloqueado é tempo em que o socket IPC do módulo não é
drenado. O `central` é single-thread: quando ele bloqueia escrevendo para o `viewer_3D`, **o
barramento inteiro para**, e todo mundo que publica — inclusive o `playback` — congela junto.

Numa sessão Wayland, com a janela do `viewer_3D` ocluída, o compositor deixa de mandar eventos
de presente e a espera do swap passa de 1 s por quadro. Daí as travadas de ~1 s.

**Como foi isolado.** Instrumentando o `playback` por fora (uma sonda que manda
`CARMEN_PLAYBACK_COMMAND_PLAY` e assina `carmen_playback_info_message`, medindo por segundo a
taxa real de reprodução e o **maior silêncio entre mensagens**), e ligando/desligando módulos
com `proccontrol_setgroup` / `proccontrol_setmodule`. Janelas de 40 s:

| condição | travadas > 0,5 s | pior travada |
|---|---|---|
| stack completa | 9 | 2,03 s |
| grupo `interface` desligado | 0 | 0,12 s |
| só os dois `camera_viewer` desligados | 13 | 2,06 s |
| `navigator_gui2` desligado | 21 | 2,05 s |
| **`viewer_3D` desligado** | **0** | 0,10 s |

Confirmação do mecanismo: o `wchan` do `viewer_3D` ficava em
`drm_syncobj_array_wait_timeout` (espera de vsync) com 0,0% de CPU, e amostrando
`ss -tnpm` a 10 Hz o RecvQ do socket dele ficava **travado em 22 KB** por centenas de
milissegundos — contrapressão de IPC de verdade.

**Hipótese descartada com medição:** I/O síncrono das nuvens do lidar em
`carmen_string_and_file_to_variable_velodyne_scan_message()` (`src/logger/readlog.cpp`), que
faz `fopen` + 3 `fread` por shot + `fclose` dentro do laço principal do playback. Com os
arquivos do lidar em page cache, o `/proc/<pid>/io` do `playback` marcava **0 KB/s de
`read_bytes`** durante as travadas e o engasgo continuava idêntico; `folio_wait_bit_common`
apareceu em 5 de 1435 amostras de `wchan` (0,3%). O disco está fora — o padrão de leitura
continua feio, mas não é o que causa o engasgo.

**Correção.** Em `src/viewer_3D/Window.cpp`, uma função `desliga_vsync()` chamada logo após o
`glXMakeCurrent()` em `initWindow()`. Ela tenta as três extensões de controle de swap, da mais
específica para a mais antiga — `GLX_EXT_swap_control` (`glXSwapIntervalEXT`), depois
`glXSwapIntervalMESA`, depois `glXSwapIntervalSGI` — e, se nenhuma existir, segue com o vsync
ligado (não vale abortar o módulo por causa disso). O módulo já limita a taxa de desenho em
40 Hz por conta própria, então o vsync não acrescentava nada.

É a mesma correção que um fork deste código-base já tinha.

**Verificação.** Stack completa, `viewer_3D` subido pelo `proccontrol` com o binário
recompilado e sem variável de ambiente: **0 travadas > 0,5 s em 40 s, taxa 1,000x, maior
silêncio 0,11 s** (que é o espaçamento normal entre mensagens). O `wchan` do módulo passou de
`drm_syncobj_array_wait_timeout` para `poll_schedule_timeout` — esperando IPC, como deveria.

**Onde mais isso pode morder.** Qualquer módulo que desenhe dentro do handler de timer do IPC
e faça swap com vsync tem o mesmo problema latente numa sessão Wayland. Se aparecer engasgo
parecido, o teste rápido é rodar o módulo suspeito com `__GL_SYNC_TO_VBLANK=0 vblank_mode=0` e
ver se some.

### Achados desta sessão que **não** viraram correção

**O terminal do `proccontrol` não mostra erro de módulo.** O `proccontrol` faz `dup2` do
stdout/stderr de cada módulo para um pipe (`src/proccontrol/proccontrol.c`) e publica o
conteúdo por IPC; o terminal só mostra as linhas `Spawned`/`Killing` dele mesmo. Para ver o
erro de verdade: `./proccontrol_viewoutput` em outro terminal, a janela de output do
`proccontrol_gui`, ou rodar o módulo na mão.

**`joystick_vehicle`: botão de ativação errado.** O código testa `joystick.buttons[8]`, mas a
mensagem na tela manda apertar START. Num gamepad Xbox 360 pelo driver `xpad` (8 eixos, 11
botões) os índices são 6 = SELECT/Back, 7 = START, **8 = BTN_MODE, o botão redondo do meio**.
Apertando START nada acontece: `joystick_activated` fica 0 e o módulo não publica nem o estado
`Free_Running` nem o motion command. O teste é de nível, não de borda, então um toque longo
pode alternar duas vezes. Para diagnosticar:
`./joystick_vehicle -direct_v_and_phi_mode on -show_state on`.

**`joystick_vehicle` morre se o `/dev/input/js0` sumir** (`carmen_die`, em `main()`). Um tranco
no cabo do controle vira loop de respawn do `proccontrol`.

**`util_publish_initial_pose` / `util_publish_final_goal` nunca saem** — ficam em *"tecle
qualquer tecla para terminar"*. Sob o `proccontrol` eles não têm terminal, então sobrevivem ao
`kill` e vazam um processo por rodada.

**A cadeia de controle da simulação está sadia.** Publicando `carmen_base_ackerman_motion_command`
na mão contra `simulator_ackerman`, o carro anda como esperado. Vale saber que o simulador só
aplica `target_v` se `behavior_selector_low_level_state != Stopped`
(`src/simulator_ackerman/simulator_ackerman_simulation.c`) — `target_phi` ele aplica sempre. Se
um dia o carro esterçar mas não andar, é esse o ponto.

### 46. Novo módulo `src/pi_nit` — detecção de pessoas em Raspberry Pi 5 + Hailo-8L

Módulo trazido de outra árvore de código, onde foi escrito. Tira a inferência 2D da GPU do PC
e a coloca num acelerador dedicado de ~5 W: o cliente C++ assina a câmera, manda o frame por
ZMQ para um Raspberry Pi 5 com Hailo-8L, recebe as caixas de volta e publica a
`neural_detector_message`. A documentação completa de funcionamento, parâmetros e testes está
em `src/pi_nit/README.md`, `src/pi_nit/COMO_TESTAR.md` e `src/pi_nit/HARDWARE.md`.

O que o porte exigiu:

**a) A interface `neural_detector` veio junto.** `neural_detector_message` é o formato que o
`pi_nit` publica, e nesta árvore **não existe** módulo dono dela. Os três arquivos
(`neural_detector_messages.h`, `neural_detector_interface.h`, `neural_detector_interface.c`)
foram trazidos para dentro de `src/pi_nit/` e são compilados e exportados por ele como
`libneural_detector_interface.a`. Está comentado no `Makefile` do módulo: se um dia entrar um
módulo dono da interface, é só apagar os três arquivos e voltar a linkar com
`-lneural_detector_interface`.

**b) `process_image()` tem assinatura diferente aqui.** A árvore de origem devolve as
intrínsecas corrigidas pela retificação:

```c
int process_image(camera_message *, int image_index, char *camera_name, int correct,
                  cv::Mat &src_image, double &new_fx, double &new_fy, double &new_cu, double &new_cv);
```

A daqui (`src/camera_drivers/camera_drivers_process_image.hpp`) devolve só a imagem, com cinco
parâmetros. O `pi_nit` declarava as quatro intrínsecas e **não as usava** — trabalha em pixels
da imagem original —, então a chamada foi encurtada para a assinatura desta árvore. **Nenhuma
alteração no `camera_drivers`.**

**c) Nomes.** O prefixo de identificadores da árvore de origem mapeia 1:1 para o do CARMEN,
então a conversão foi mecânica: identificadores e macros passaram a `carmen_*` / `CARMEN_*`, os
includes passaram a `<carmen/...>`, a variável de ambiente do home da árvore passou a
`CARMEN_HOME`, e a prosa das quatro documentações foi reescrita. A mensagem de status ficou
`carmen_pi_nit_status_message` / `CARMEN_PI_NIT_STATUS_NAME`.

**d) O que não foi copiado.** Os pesos YOLO (`pi_nit_server/tools/*.pt`, ~147 MB) e o vídeo de
exemplo (`data/pi_nit/pedestres.avi`, 7,8 MB) — binários não entram no repositório. O
`pi_nit_server/download_model.sh` já existia para baixar os pesos. Também ficaram de fora os
artefatos de build e um arquivo vazio de nome inválido (`pi_nit_server/image/imagem:`). Foi
criado `src/pi_nit/.gitignore` para os três binários do módulo e para `*.pt`/`*.hef`/`*.onnx`/`venv/`,
e as três entradas `bin/pi_nit_*` foram para o `.gitignore` da raiz.

**e) Documentação do que não existe aqui.** O `README.md` do módulo ganhou uma seção
*"Leia primeiro: o que existe nesta árvore"* deixando explícito que os consumidores citados ao
longo do texto (`neural_image_tracker`, `multiple_object_tracker`, `image_path_projector`) são
da árvore de origem e **não fazem parte desta** — as referências a arquivos e linhas deles
servem para explicar de onde vem o formato da mensagem e o que um consumidor faz com ela.

`pi_nit` foi acrescentado ao `PACKAGES` do `src/Makefile` (junto de `pi_imu` e `pi_camera`).
Dependência nova do sistema: **`libzmq3-dev`** (a `libzmq` 4.3.5 do 26.04 serve). Compila
limpo e gera `pi_nit_client_driver`, `pi_nit_link_test` e `pi_nit_camera_publisher` em `bin/`.

#### Validação em execução (2026-08-20)

A cadeia inteira foi exercitada nesta máquina, **sem o Raspberry**, com o servidor em modo
`--dummy` no loopback. O roteiro virou o **Teste 0** do `src/pi_nit/COMO_TESTAR.md`.

| camada | como foi exercitada | resultado |
|---|---|---|
| enlace ZMQ (C++ ↔ Python) | `pi_nit_link_test 127.0.0.1 -frames 30 -fps 15` | enviados 30, recebidos 29, 14,5 fps de retorno |
| protocolo binário | o mesmo teste, com o servidor decodificando | caixa devolvida e decodificada em todos os frames |
| `camera_message` → cliente | `pi_nit_camera_publisher` tocando `pedestres.avi` em loop | 149 frames por câmera em 15 s, sem descarte |
| batch de 3 câmeras | 3 publishers (mensagens 3, 4 e 5) + 1 cliente com as 3 | `enviados 149/150/149, recebidos 149/149/149` |
| letterbox e volta das coordenadas | caixa fixa do dummy remapeada | `307, 96, 153×479` em pixels da imagem original de 768×576 |
| tracking | `-track` no padrão | `track_id` estável em 1 ao longo da sequência |
| publicação IPC | `print_ipc_message neural_detector_message_{3,4,5}_name` | as três mensagens saem, com `obj_id -1` (convenção do MOT) |
| stream do viewer | SUB em `tcp://127.0.0.1:5562` | 149 quadros em 5 s (3 partes: id, JPEG, deteções) |
| cliente Python | `tools/test_client.py --simulate-cameras 3 --loop` | roda e imprime as deteções, `descartados no envio 0` |

Duas coisas do 26.04 que o roteiro original não previa, e que ficaram documentadas no
`COMO_TESTAR.md`:

- **O venv precisa de `--system-site-packages`.** O Python do 26.04 é o **3.14** e não há wheel
  de `opencv-python-headless` para ele; o `requirements.txt` só se resolve reaproveitando o
  `python3-opencv` e o `python3-numpy` do sistema. Sobra o `pyzmq`, que tem wheel. (A árvore de
  origem supunha um venv que já existia lá, de outro módulo.)
- **A URL do vídeo de exemplo estava errada** — `github.com/opencv/opencv/raw/master/...`
  devolve HTML. Trocada por `raw.githubusercontent.com/opencv/opencv/4.x/...`.

O `data/pi_nit/` inteiro (vídeo, pesos e venv) entrou no `.gitignore` da raiz: é tudo baixado
por script.

⚠️ **O que continua sem validação:** a **inferência de verdade**. O modo `--dummy` devolve uma
caixa fixa, então nada além do caminho de dados foi provado. Faltam os dois:

- o **Hailo-8L no Raspberry** (Testes 2 e 2b) — precisa do hardware na rede;
- o backend **`cpu`** (Teste 1), que daria detecções reais no PC, mas exige `ultralytics` +
  `torch` (~3 GB) no venv. Não instalado.

O que já estava verificado antes: compila e linka sem aviso, os `static_assert` de layout do
protocolo passam (o que garante que o header C++ e o `pi_nit_protocol.py` continuam de acordo),
os binários sobem e o `include/carmen` não foi sequestrado.

### 47. `traffic_light`: a API C do OpenCV, escondida atrás do `nvcc`

Relatado como "erro de OpenCV no `traffic_light`" numa **outra máquina, na mesma branch**.
A causa da diferença entre as duas máquinas está no `Makefile` do módulo:

```make
ifneq (, $(shell which nvcc))
TARGETS += traffic_light log_generate_images
endif
```

Numa máquina **sem CUDA** (como a do desenvolvimento) o alvo é só
`libtraffic_light_interface.a`, e o `make` nunca chega a compilar
`traffic_light_main.cpp` nem `log_generate_images.cpp`. Numa máquina **com CUDA** ele compila
os dois — e aí aparecem os erros de OpenCV 4 que estavam guardados ali desde sempre. O mesmo
vale para `TLightStateRecog/tlight_vgram.cpp` e `traffic_light_view.cpp`, que dependem da
`libwnn` (`HAS_WNN`, item 27).

Quatro arquivos, todos com o mesmo remédio — `#include <carmen/opencv_c_compat.h>`:

| arquivo | o que quebrava | quando compila |
|---|---|---|
| `log_generate_images.cpp` | `#include <opencv/cv.h>` (diretório removido no OpenCV 4); `IplImage`, `cvCreateImage`, `cvCvtColor`, `CV_BGR2RGB`; `cvSaveImage` saiu do binário | com `nvcc` |
| `traffic_light_main.cpp` | `CvPoint`, `CV_BGR2GRAY`, `CV_INTER_CUBIC` | com `nvcc` |
| `traffic_light_view.cpp` | `CvPoint` | com `libwnn` |
| `TLightStateRecog/tlight_vgram.cpp` | `CV_BGR2YCrCb`, `CV_YCrCb2BGR` | com `libwnn` |

O `log_generate_images.cpp` tinha o `#if CV_MAJOR_VERSION == 2 / #elif >= 3` do item 16 já
aplicado, mas o ramo do `>= 3` continuava incluindo `<opencv/cv.h>` — o `==3` → `>=3` só
consertou a *escolha* do ramo, não o conteúdo dele. Vale procurar esse resto em quem ainda usa
o par legacy/`opencv/cv.h`.

**Verificação.** Como esta máquina não tem `nvcc` nem `libwnn`, os arquivos foram compilados
na mão, com as mesmas flags do `Makefile`:

```bash
g++ -c -std=c++11 -I$CARMEN_HOME/include $(pkg-config --cflags opencv4) \
    -I/usr/local/include/bullet/ log_generate_images.cpp -o /tmp/x.o     # ok
g++ -fsyntax-only -std=c++11 -I$CARMEN_HOME/include $(pkg-config --cflags opencv4) \
    $(pkg-config --cflags gtk+-2.0) traffic_light_view.cpp                # ok
```

Antes da correção os três reproduziam o erro; depois, os três compilam limpo. O
`tlight_vgram.cpp` **não pôde ser compilado aqui** — o `tlight_vgram.h` inclui
`wnn/VgRamNeuron.h`, que vem da `libwnn`. A correção dele é a mesma dos outros, mas só será
provada numa máquina que tenha a lib.

`make` no módulo continua saindo 0 nesta máquina, e o `include/carmen` não foi sequestrado.

⚠️ Só existem **dois** módulos com alvo condicionado ao `nvcc`: `traffic_light` e
`road_mapper`. O do `road_mapper` é duplamente condicionado (`nvcc` **e** `CAFFE_ENET_HOME`) e
o arquivo dele já tinha sido corrigido. Fora esses dois, uma máquina com CUDA não compila nada
a mais do que esta.
