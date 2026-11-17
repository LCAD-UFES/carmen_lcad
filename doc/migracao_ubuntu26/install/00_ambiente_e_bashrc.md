# 0. Ambiente: variáveis do `~/.bashrc` e `configure`

Primeiro passo da instalação, antes de qualquer `apt-get` ou `make`. Sem `CARMEN_HOME` o
build nem começa: todo `Makefile` do repo faz `include $(CARMEN_HOME)/src/Makefile.conf`.

## 1. Bloco a adicionar no final do `~/.bashrc`

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

Depois: `source ~/.bashrc` (ou abrir um terminal novo) e conferir:

```bash
echo $CARMEN_HOME       # /home/<usuário>/carmen_lcad
ls $CARMEN_HOME/src/Makefile.conf
```

### O que mudou em relação ao tutorial de Ubuntu 16.04/20.04

| Linha antiga | Agora |
|---|---|
| `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:...` (sem o `$CARMEN_HOME/lib`) | `$CARMEN_HOME/lib` **na frente** — ver aviso abaixo |
| Bloco `#Cuda` com `/usr/local/cuda-9.1` | Só se a máquina tiver CUDA instalado, e com a versão real (ver `04_bibliotecas_sem_pacote_apt.md`) |
| Bloco `#Caffe ENet` (`CAFFE_ENET_HOME`, `PYTHONPATH`) | Só para os módulos de segmentação semântica com Caffe, que não fazem parte do `make` principal |
| `python`/`python-numpy` (Python 2) | Python 2 não existe mais no Ubuntu 26.04; tudo é `python3` |

### ⚠️ Se esta máquina também tem outro repositório do LCAD

Há forks do CARMEN em uso no laboratório, e **os dois geram bibliotecas com os mesmos nomes**
(`libglobal.so`, `libipc.a`, `libparam_interface.so`, ...). Se o `~/.bashrc` tiver os dois
blocos (`<fork>/lib` e `$CARMEN_HOME/lib`) no `LD_LIBRARY_PATH`, **quem aparecer
primeiro ganha em tempo de execução** — e um binário do CARMEN pode acabar carregando a lib
do fork (ou vice-versa), com sintomas confusos (símbolo faltando, mensagem IPC com layout
diferente, crash na subida do módulo).

Por isso o bloco acima põe `$CARMEN_HOME/lib` **na frente** da variável. Se for trabalhar
principalmente com o fork, inverta — o importante é ter consciência de que essa ordem
decide qual repo "vence".

Só o `LD_LIBRARY_PATH` tem esse problema; o `PATH` (binários) é mais visível: `which
navigator_gui2` mostra de qual repo veio.

## 2. `configure`

O CARMEN, diferente de forks mais novos, ainda usa o `src/configure` (script Perl) para gerar o
`src/Makefile.vars`, que o `Makefile.conf` inclui. Rodar uma vez, dentro de `src/`:

```bash
cd $CARMEN_HOME/src
./configure --nojava --nocuda --noqt3
```

Perguntas do script e o que responder nesta migração:

| Pergunta | Resposta usada | Por quê |
|---|---|---|
| `Should the C++ tools be installed for CARMEN [Y/n]` | Enter (Y) | O build principal usa C++ em quase tudo |
| `Should Python Bindings be installed [y/N]` | Enter (N) | Bindings SWIG antigos (Python 2-era); não são necessários e são fonte de erro |
| `Should the old laser server be used [y/N]` | Enter (N) | `sensors/laser_new` é o driver atual (o `configure` cria o symlink `src/laser -> sensors/laser_new`) |
| `Install path [/usr/local/]` | Enter | Não é usado por nenhuma regra do build |
| `Robot numbers [*]` | Enter (todos) | Só decide quais módulos de base são compilados |

Flags usadas e por quê:
- `--nojava`: as bindings Java são de uma era pré-Java 8 e não são usadas pelo LCAD.
- `--nocuda`: liga/desliga `-lcuda -lcudart` global. Ligar só depois de confirmar que existe
  CUDA compatível com o GCC 15 do 26.04 (ver `04_bibliotecas_sem_pacote_apt.md`).
- `--noqt3`: Qt3 não existe há mais de uma década.

Resultado esperado em `src/Makefile.vars` (arquivo gerado, **não** versionado):

```make
CARMEN_HOME = /home/<usuário>/carmen_lcad
GTK_CONFIG = pkg-config gtk+-2.0
NO_TCPD = 1
NO_LIBART = 1
NO_CANLIB = 1
NO_PYTHON = 1
PROCESSOR = x86_64
NO_CUDA = 1
NO_JAVA = 1
...
```

`NO_TCPD = 1` aparece porque o `tcpd.h` (libwrap) não é achado pelo teste do `configure`
mesmo com `libwrap0-dev` instalado — é só o controle de acesso opcional do IPC, não bloqueia
nada.

Próximo passo: `01_ferramentas_de_build.md`.
