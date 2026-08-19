# 1. Ferramentas de build

Numa imagem nova do Ubuntu 26.04 nada disso vem instalado:

```bash
sudo apt-get update

sudo apt-get install -y \
    build-essential gcc g++ make cmake cmake-curses-gui \
    git pkg-config doxygen byacc flex swig perl \
    wget tcsh vim gnuplot-qt plocate
```

Conferir as versões:

```bash
gcc --version      # esperado: 15.x  (Ubuntu 26.04 = gcc-15)
g++ --version      # idem
make --version
cmake --version    # esperado: 4.x
perl --version     # o src/configure do CARMEN é um script Perl
```

Diferenças em relação ao tutorial de 16.04/20.04:

| Tutorial antigo | Ubuntu 26.04 |
|---|---|
| `mlocate` | **NOT-FOUND** — usar `plocate` (mesmo comando `updatedb`/`locate`) |
| `git-core` | Pacote virtual antigo; `git` já cobre |
| `cmake-qt-gui` | Ainda existe, mas é opcional (só a GUI do cmake) |

## ⚠️ O `gcc` padrão precisa ser o gcc-15

Todas as bibliotecas do apt no 26.04 (`libopencv-dev`, `libpcl-dev`, `libboost-all-dev`, ...)
foram compiladas pela Canonical com **gcc-15**. Se o `/usr/bin/gcc` desta máquina estiver
apontando para uma versão antiga via `update-alternatives` (é comum alguém fixar gcc-11 por
causa de CUDA), o build compila mas **quebra no link**, com erros do tipo:

```
undefined reference to `std::condition_variable::wait(...)@GLIBCXX_3.4.30'
```

Isso **não** é falta de pacote nem bug do CARMEN — é ABI incompatível entre o compilador
local e as `.so` do apt. Conferir:

```bash
gcc --version
update-alternatives --list gcc     # se listar só gcc-11, é este o problema
```

Duas formas de resolver:

1. **Tornar o gcc-15 o padrão do sistema** (recomendado se não for mexer com CUDA agora):
   ```bash
   sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-15 100
   sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-15 100
   sudo update-alternatives --set gcc /usr/bin/gcc-15
   sudo update-alternatives --set g++ /usr/bin/g++-15
   ```
   Depois disso, rodar `make clean` em `$CARMEN_HOME/src` antes do próximo build: `.o`/`.a`
   gerados com o compilador antigo têm bytecode LTO incompatível
   (`lto1: fatal error: bytecode stream ... generated with LTO version ...`).

2. **Sem mexer no padrão do sistema**, passando o compilador a cada build:
   ```bash
   make -C $CARMEN_HOME/src CC=gcc-15 CXX=g++-15 -j$(nproc)
   ```

Nesta migração a máquina já estava com gcc-15 como padrão (`gcc (Ubuntu 15.2.0-16ubuntu1)`),
então o problema não apareceu — mas ele é o primeiro suspeito se um erro de link
`@GLIBCXX_*` aparecer numa máquina nova.

Próximo passo: `02_boost.md`.
