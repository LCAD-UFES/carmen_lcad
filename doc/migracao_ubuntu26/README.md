# Migração do CARMEN LCAD para Ubuntu 26.04

Esta pasta reúne **tudo relacionado à migração do build do `carmen_lcad` de Ubuntu
16.04/20.04 para Ubuntu 26.04 LTS ("resolute")**: o que foi alterado no código e nos
Makefiles, por quê, e o que precisa ser instalado no sistema.

Branch de trabalho: **`migracao_ubuntu26`** (criada a partir de `ubuntu26`). **Nada foi
commitado nem pushed** — as alterações estão só no working tree local, para revisão
(`git diff`) antes de decidir o que entra no commit.

## Estrutura

- **`CHANGELOG.md`** — log, item a item, de toda alteração feita em Makefiles/código por causa
  da migração (arquivo, o quê, por quê). É o documento principal.
- **`AUDIT_DEPENDENCIAS.md`** — auditoria das dependências externas: nome/versão do pacote no
  Ubuntu 26.04, o que mudou de nome, o que sumiu do apt, e o que continua sem pacote.
- **`CHECKLIST_COMMIT.md`** — o que está no working tree agora: o que deve subir no commit, o
  que é artefato local de build e não deve subir, e os comandos prontos.
- **`install/`** — o roteiro de instalação numa máquina nova, em ordem:
  - `00_ambiente_e_bashrc.md` — variáveis do `~/.bashrc` (`CARMEN_HOME` etc.) e `configure`
  - `01_ferramentas_de_build.md` — compilador, make, cmake (e o cuidado com a versão do gcc)
  - `02_boost.md`
  - `03_opencv_pcl_g2o_dlib_eigen.md`
  - `04_bibliotecas_sem_pacote_apt.md` — demais pacotes apt, GtkGLExt, SDKs de hardware, CUDA
  - `05_pendencias_conhecidas.md` — o que ficou pendente e por quê

## Como usar (máquina nova)

```bash
git clone https://github.com/LCAD-UFES/carmen_lcad.git ~/carmen_lcad
# 1) doc/migracao_ubuntu26/install/00 → bashrc + configure
# 2) install/01 a 04 → pacotes
cd $CARMEN_HOME/src && ./configure --nojava --nocuda --noqt3
make -j$(nproc)
```

## Contexto: há um fork mais novo deste mesmo código

Existe um fork do `carmen_lcad`, mais novo, que mantém a mesma
estrutura de build (`Makefile.conf`/`Makefile.rules`, `IFLAGS`/`LFLAGS`, `PUBLIC_*`), então as
correções são compatíveis entre os dois. Boa parte dos problemas desta migração já tinha sido
diagnosticada lá, e o mesmo raciocínio foi aplicado aqui — mas **toda alteração deste
repositório foi feita e testada nele mesmo**, com o `make` real na árvore do `carmen_lcad`;
nada foi copiado às cegas. Onde os dois divergem (o CARMEN ainda usa `src/configure` e
`Makefile.vars`, tem mais módulos e mantém código antigo que o fork já removeu), a correção
foi feita do zero para o CARMEN.

## Resumo das causas-raiz

| Problema | Causa | Correção |
|---|---|---|
| Build pede um Boost "especial" | 28 Makefiles com `-I/-L /usr/local/carmen_boost` hardcoded (Boost 1.61 compilado à mão do tutorial antigo) | Flags removidas — usa o Boost do sistema (`libboost-all-dev`) |
| `cannot find -lboost_thread-mt` | Sufixo `-mt` é convenção morta do Boost | `-lboost_thread-mt` → `-lboost_thread` (94 Makefiles) |
| `cannot find -lboost_signals` / `-lboost_system` | Signals v1 foi removido do Boost; System virou header-only sem `.so` | Flags removidas; o uso real da API em `libtf` foi portado para `Boost.Signals2` |
| `pkg-config opencv` falha | Distros modernas só trazem `opencv4.pc` | Todas as chamadas com fallback `opencv4` → `opencv` (201 Makefiles) |
| `opencv2/*.hpp: No such file` | O pacote do 26.04 põe os headers em `/usr/include/opencv4` (exige `-I`), e 38 módulos linkavam OpenCV sem nunca pedir `--cflags` | `IFLAGS += pkg-config --cflags opencv4 ...` nos 38 |
| `opencv/cv.h: No such file` | Headers da era OpenCV 1/2 removidos | 89 arquivos migrados para `opencv2/...` (API C ou C++ conforme o uso real) |
| Headers do PCL/VTK/Bullet/cs.h não encontrados | Makefiles fixam paths de instalação manual em `/usr/local` | Auto-detecção aditiva em `src/Makefile.conf` |
| `#error C++ standard too low` (PCL) | PCL 1.15 exige C++17 | 44 Makefiles subidos para `-std=c++17` |
| `Boost.Math requires C++14` | Boost 1.90 via `libtf` | 38 Makefiles + `libtf` subidos para `-std=c++14` |
| Erros do GCC 15 (`template with C linkage`, K&R, ponteiro incompatível) | O que era aviso virou erro | Correções pontuais — ver `CHANGELOG.md` |
| `carmen/web_cam_interface.h: No such file` | Headers públicos de módulos que não estavam em nenhuma lista do `src/Makefile` | `sensors/web_cam` e `sensors/ultrasonic` adicionados ao `PACKAGES` |

## Avisos práticos

⚠️ **Nunca rode dois `make` simultâneos na mesma árvore** — não há lock: um processo compila
o `.o` enquanto o outro linka o arquivo pela metade, gerando erros de link não-determinísticos
que parecem bug de código. Antes de investigar erro de link estranho:
`ps -eo args | grep '[m]ake -j'`.

⚠️ **O `make` do topo não é tolerante a falha.** O `RECURSE` do `src/Makefile` é um `for` do
shell com `exit -1` embutido: o primeiro módulo que falhar aborta tudo, e nem `make -k` ajuda
(quem aborta é o shell, não o Make). Para pular um módulo problemático e testar o resto, tire
o nome dele da lista `PACKAGES` em `src/Makefile`.

⚠️ **Erro de link "undefined reference" para algo que está claramente no código-fonte**:
quase sempre é `.o` velho de uma tentativa anterior. `rm -f *.o *.a Makefile.depend` no módulo
antes de investigar a fundo.
