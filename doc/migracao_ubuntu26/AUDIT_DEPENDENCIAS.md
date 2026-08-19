# Auditoria de dependências externas — Ubuntu 16.04/20.04 → 26.04

Base: os tutoriais `doc/README_Installing_Carmen_LCAD_on_Ubuntu_16.04.md` e a lista
equivalente de 20.04. Toda versão candidata abaixo foi checada com `apt-cache policy <pacote>`
contra os repositórios reais do Ubuntu 26.04 "resolute" — não é chute. **NOT-FOUND** = o
pacote não existe mais com esse nome (renomeado, incorporado a outro, ou removido do Ubuntu).

## Boa notícia: quase tudo que o tutorial mandava compilar na mão hoje tem pacote apt

No 16.04/20.04 o roteiro pedia compilar do zero: OpenCV 3.2 (e depois 4.5.5), PCL 1.8 (com
patches manuais), g2o (clone do fork do LCAD), dlib, FANN (SourceForge), Boost 1.61 com prefixo
customizado. Nada disso é mais necessário:

| Lib | Pacote apt (26.04) | Versão candidata | Observação |
|---|---|---|---|
| Boost | `libboost-all-dev` | 1.90.0.1ubuntu3 | Sem prefixo customizado — ver `install/02_boost.md` |
| OpenCV | `libopencv-dev` | 4.10.0+dfsg-7ubuntu5 | Só OpenCV 4; não existe mais pacote de OpenCV 3 |
| PCL | `libpcl-dev` | 1.15.1+dfsg-2 | Headers em `/usr/include/pcl-1.15`; **exige C++17** |
| g2o | `libg2o-dev` | 0~20230806-5 | Não precisa mais clonar `LCAD-UFES/g2o-1` |
| dlib | `libdlib-dev` | 20.0+dfsg-2 | Não precisa mais compilar `davisking/dlib` |
| Eigen3 | `libeigen3-dev` | 3.4.0-5 | Path padrão `/usr/include/eigen3`, sem mudança |
| GSL | `libgsl-dev` | 2.8+dfsg-6 | — |
| Bullet | `libbullet-dev` | 3.24+dfsg-5 | Headers em `/usr/include/bullet` (antes `/usr/local/include/bullet`) |
| SuiteSparse | `libsuitesparse-dev` | 1:7.12.2+dfsg-1ubuntu1 | Traz o `cs.h` que o g2o/CSparse precisa |
| VTK | `libvtk9-dev` | 9.5.2+dfsg4-3ubuntu1 | Vem junto como dependência do `libpcl-dev` |
| FANN | `libfann-dev` | 2.2.0+ds-9 | Não precisa baixar do SourceForge |
| LAPACKE | `liblapacke-dev` | 3.12.1-7ubuntu1 | — |
| HDF5 | `libhdf5-dev` | 1.14.6+repack-2 | — |
| FLANN | `libflann-dev` | 1.9.2+dfsg-7 | — |
| Qhull | `libqhull-dev` | 2020.2-8 | — |
| Qt | `qtbase5-dev` | 5.15.18+dfsg-1ubuntu1 | Substitui o Qt4 do tutorial (ver abaixo) |

## Pacotes que sumiram / mudaram de nome

| Pacote (tutorial antigo) | Status em 26.04 | O que usar |
|---|---|---|
| `libqt4-dev`, `qt4-qmake` | **NOT-FOUND** — Qt4 saiu do Ubuntu | `qtbase5-dev`; ver `CHANGELOG.md` item 13 |
| `libncurses5`, `libncurses5-dev` | **NOT-FOUND** | `libncurses-dev` (ABI atual) |
| `libgtkglext1`, `libgtkglext1-dev`, `libgtkglextmm-x11-1.2-dev` | **NOT-FOUND** | Compilar da fonte — ver `install/04`, seção 4.3 |
| `libglade2-0`, `libglade2-dev`, `libglademm-2.4-dev` | **NOT-FOUND** | Não é preciso: o código já usa `GtkBuilder` |
| `freeglut3` (runtime) | NOT-FOUND | `freeglut3-dev` traz o runtime junto |
| `libcurl4-nss-dev` | NOT-FOUND | `libcurl4-openssl-dev` |
| `libdc1394-22`, `libdc1394-22-dev` | NOT-FOUND | `libdc1394-dev`/`libdc1394-utils` (só câmera FireWire) |
| `libjasper1`, `libjasper-dev` | NOT-FOUND | Só servia para compilar OpenCV 3 à mão — irrelevante |
| `libvtk5-dev`, `libvtk6-dev` | NOT-FOUND | `libvtk9-dev` |
| `libflann1.8` | NOT-FOUND | `libflann-dev` |
| `libboost1.58-all-dev` | NOT-FOUND | `libboost-all-dev` (1.90) |
| `mlocate` | NOT-FOUND | `plocate` (mesmos comandos `updatedb`/`locate`) |
| `git-core` | NOT-FOUND | `git` |
| `python-numpy`, `python-dev`, `python-pip` (Python 2) | NOT-FOUND | `python3-numpy`, `python3-dev`, `python3-pip` |
| `oracle-java8-installer` (PPA webupd8) | PPA morto | Não usado — `configure --nojava` |

## Removido do próprio Boost (não é questão de nome de pacote)

- **`Boost.Signals`** (v1, `-lboost_signals`) — removido do Boost faz anos; não existe
  `libboost-signals-dev` em distro recente nenhuma. O único uso real (em `sharedlib/libtf`)
  foi portado para `Boost.Signals2` (header-only). Ver `CHANGELOG.md` item 3.
- **`Boost.System`** (`-lboost_system`) — header-only; o `libboost-all-dev` do 26.04 não gera
  mais a `.so` de compatibilidade. Flag removida de 117 Makefiles (item 4).
- **Sufixo `-mt`** (`-lboost_thread-mt`) — convenção abandonada há mais de uma década (item 2).

## Requisitos de padrão C++ que mudaram

| Lib | Exige | Efeito no repo |
|---|---|---|
| PCL 1.15 | C++17 | `#error C++ standard too low` vindo do `pcl_config.h`; 44 Makefiles subidos (item 11) |
| Boost 1.90 (`boost/math`, via `libtf`) | C++14 | `#warning "Boost.Math requires C++14"` + erro `'is_final' has not been declared in 'std'`; 38 Makefiles + o `libtf` (item 11) |

## Mudanças do compilador (GCC 15) que quebram código antigo

Não são dependências, mas causam a mesma classe de erro "só no Ubuntu novo":

| Padrão | Antes | GCC 15 |
|---|---|---|
| Ponteiro de função K&R `void (*f)();` | Aviso | **Erro** (`too many arguments to function`) — item 18 |
| Ponteiro incompatível (`gzFile *` vs `gzFile`) | Aviso | **Erro** — item 9 |
| `implicit-function-declaration` | Aviso | **Erro** |
| `omp.h` dentro de `extern "C"` | Compilava | **Erro** (`template with C linkage`) — item 10 |

## Coisas sem pacote apt — continuam sendo instaladas manualmente

Iguais ao tutorial original:

- **GtkGLExt** — sem pacote em nenhuma distro atual; compilar da fonte (`install/04`, 4.3).
  É pré-requisito da GUI principal (`navigator_gui2`) e de ~20 módulos.
- **Kvaser CAN SDK** (`linuxcan`) — driver do fabricante.
- **PointGrey/FLIR Flycapture 2** — SDK proprietário de câmera (`bumblebee_basic`).
- **Ouster SDK antigo** (`ouster_example`) — para `sensors/ouster_sdk_driver_5-2`.
- **libwnn** (`filipemtz/libwnn`) — repo do LCAD, sem pacote.
- **MAE** (`MAEHOME`) — usado pelo módulo `tracker`.

## O que ainda não foi auditado

- `src/ros1_sc_lio_sam/` e `src/ros1-lt-mapper/` (bridges ROS, build `catkin` à parte).
- `sharedlib/darknet*`, `sharedlib/ENet`, `src/semantic_segmentation/caffe-segnet` — código de
  terceiros vendorizado, fora do `make` principal; ainda usam constantes antigas do OpenCV e
  APIs de CUDA/Caffe de 2018.
- `ubuntu_packages/` na raiz do repo — `.deb`s e SDKs antigos (`boost_1_61_0.tar.xz`,
  `flycapture2-2.5.3.4`, `imlib`, `zlib1g`). O `boost_1_61_0.tar.xz` era do fluxo manual do
  Boost, que não é mais necessário depois do item 1 do `CHANGELOG.md`.
