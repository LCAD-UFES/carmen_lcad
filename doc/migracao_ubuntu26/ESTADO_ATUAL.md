# Estado atual da migração — build completo, runtime iniciado

Atualizado em 2026-08-19, na branch `ubuntu26`.
**Nada commitado** (a pedido) — tudo está no working tree.

## O build compila inteiro

| Fase | Situação |
|---|---|
| `export` (headers públicos) | ✅ completa |
| `phase1` (bibliotecas) | ✅ completa — 123 módulos |
| `phase2` (binários) | ✅ **completa** — 114 módulos |

`make -j$(nproc)` a partir de `$CARMEN_HOME/src` termina com `Done making binaries...` e
código de saída **0**. Rodado duas vezes seguidas para confirmar que é idempotente (a segunda
passada não recompila nada e também sai 0).

Artefatos gerados: **657** binários em `bin/`, **138** arquivos em `lib/`,
**279** headers em `include/carmen/`.

## Como reproduzir do zero (ou depois de religar a máquina)

```bash
source ~/.bashrc                 # o bloco #CARMEN já está lá
cd $CARMEN_HOME/src
git branch --show-current        # deve dizer: ubuntu26
make -j$(nproc) 2>&1 | tee /tmp/make_carmen.log
```

Não precisa rodar `./configure` de novo — o `src/Makefile.vars` já está gerado (ele não é
versionado, mas continua no disco).

O build é **incremental**. Se algum erro de link parecer sem sentido (`undefined reference`
para algo que está claramente no código), limpe o módulo antes de investigar:
```bash
cd $CARMEN_HOME/src/<modulo> && rm -f *.o *.a Makefile.depend && make
```

## Últimas correções (fecharam a `phase2`)

São os itens **38 a 42** do `CHANGELOG.md`:

| # | Módulo | Problema |
|---|---|---|
| 38 | `sharedlib/OpenJAUS/ojTorc` | `enum {...} GEAR_NUMBER;` sem `typedef` — variável global no header, `multiple definition` com `-fno-common` |
| 39 | `visual_car_tracking` | `CV_HAAR_SCALE_IMAGE` removida no OpenCV 4 → `cv::CASCADE_SCALE_IMAGE` |
| 40 | `graphslam` | `pcl::PointCloud<T>::Ptr` deixou de ser `boost::shared_ptr` (PCL ≥ 1.11) |
| 41 | `graphslam` | `Factory::registerType()` do g2o 2023 recebe `std::shared_ptr` |
| 42 | `graphslam` | `Registration::setInputCloud()` virou privada → `setInputSource()` |

⚠️ O item 38 tem uma armadilha que vale lembrar: o Makefile do OpenJAUS **não gera
`Makefile.depend`**, então editar um header de lá não recompila os `.o`. É preciso
`rm -f Build/*.o` no subprojeto antes de rebuildar.

## Runtime — começou

`./proccontrol argos/process-argos-navigate.ini` já sobe **sem nenhum `exit 127`**. O primeiro
bloqueio de runtime está no `CHANGELOG.md`, item **43**, e tinha duas causas com o mesmo código
de saída:

- `proccontrol_gui` não existia (Qt4 + Qt3Support) → **reescrito em GTK3** (item 44), no modelo de um fork deste código; o CARMEN não depende mais de Qt;
- `route_planner` / `offroad_planner` pediam `libgsl.so.23` e `libpython3.8.so.1.0` → shims de
  soname em `lib/compat` (validados símbolo a símbolo; ver pendência **i)**).

Ainda **pendentes**, por dependerem de coisas que não existem no 26.04: `task_manager`
(ROS1 Noetic) e `fastslam` (Boost 1.61 + OpenCV 3.2). Nenhum dos dois é iniciado por esse
`.ini`.

As pendências de *compilação* que ficaram de fora de propósito (módulos fora do `PACKAGES`,
`proccontrol_gui` em Qt3, bridges ROS, GtkGLExt) estão nas seções a) a h) de
`install/05_pendencias_conhecidas.md`; a seção **j)** lista o que ainda esperar do runtime.

## Runtime — sessão 2 (2026-08-20): o engasgo do `playback`

A stack de playback (`bin/argos/process-argos-playback.ini`) sobe inteira e roda. O `playback`
engasgava — travava ~1 s e recuperava a ~2x, a cada poucos segundos. **Causa achada e
corrigida:** era o `viewer_3D` bloqueando no vsync. O desenho dele roda dentro do handler de
timer do IPC, então o tempo parado no `glXSwapBuffers()` é tempo sem drenar o socket; o
`central` é single-thread e trava o barramento inteiro ao escrever para um módulo que não lê.
Numa sessão Wayland, com a janela ocluída, a espera do swap passa de 1 s por quadro. A correção
(`desliga_vsync()` em `src/viewer_3D/Window.cpp`) é o item **45** do `CHANGELOG.md`. Depois
dela: **0 travadas > 0,5 s em 40 s, taxa 1,000x**, com a stack completa.

A hipótese anterior — I/O síncrono das nuvens em `src/logger/readlog.cpp` — foi **descartada
com medição**: com os arquivos em page cache o `playback` lia 0 KB/s de disco e o engasgo
continuava idêntico.

Ficaram **abertos** (documentados no fim do `CHANGELOG.md`, sem correção):

| o quê | resumo |
|---|---|
| `joystick_vehicle` | ativa em `buttons[8]` = botão do meio do Xbox, mas a mensagem manda apertar START; e faz `carmen_die` se o `/dev/input/js0` sumir, virando loop de respawn |
| `util_publish_initial_pose` / `_final_goal` | travam em "tecle qualquer tecla"; sob o `proccontrol` vazam um processo por rodada |
| saída dos módulos | o terminal do `proccontrol` nunca mostra erro de módulo — usar `./proccontrol_viewoutput` |

## Módulo novo: `src/pi_nit` (2026-08-20)

Detecção de pessoas em Raspberry Pi 5 + Hailo-8L, trazida de outra árvore de código. Compila
limpo e gera `pi_nit_client_driver`, `pi_nit_link_test` e `pi_nit_camera_publisher`. Já está no
`PACKAGES`. Dependência nova do sistema: `libzmq3-dev`. Detalhes do porte no item **46** do
`CHANGELOG.md`; como o módulo funciona, em `src/pi_nit/README.md`.

**Validado em execução em 2026-08-20**, com o servidor em modo `--dummy` no loopback — o
roteiro virou o **Teste 0** do `src/pi_nit/COMO_TESTAR.md`. Passou a cadeia inteira: enlace ZMQ,
protocolo binário, `camera_message` → cliente, batch de 3 câmeras (149 enviados / 149 recebidos
em cada), letterbox e volta das coordenadas, tracking, publicação das três
`neural_detector_message_{3,4,5}_name` no IPC, o stream do viewer na porta 5562 e o
`tools/test_client.py`. A tabela com os números está no item **46** do `CHANGELOG.md`.

Para refazer, em dois terminais:

```bash
# 1
cd $CARMEN_HOME/src/pi_nit/pi_nit_server
$CARMEN_HOME/data/pi_nit/venv/bin/python3 pi_nit_server.py --dummy --bind 127.0.0.1 --batch-size 1
# 2
$CARMEN_HOME/bin/pi_nit_link_test 127.0.0.1 -frames 30 -fps 15
```

⚠️ **O que falta é a inferência de verdade** — o dummy devolve uma caixa fixa. São dois
caminhos, nenhum feito:

- **Hailo-8L no Raspberry** (Testes 2 e 2b do `COMO_TESTAR.md`): precisa do hardware na rede.
- **Backend `cpu`** (Teste 1): detecções reais no PC, mas exige `ultralytics` + `torch` (~3 GB)
  no venv `data/pi_nit/venv`, mais os pesos do `pi_nit_server/download_model.sh`. **Decisão
  pendente com o Miguel** — vale o download?

O venv em `data/pi_nit/venv` já existe, criado com `--system-site-packages` (o Python do 26.04
é o 3.14 e não tem wheel de `opencv-python-headless`; o `cv2` e o `numpy` vêm dos pacotes do
sistema). `data/pi_nit/` inteiro está no `.gitignore`.

## Arquivos desta migração (todos já salvos)

- `doc/migracao_ubuntu26/README.md` — visão geral e causas-raiz
- `doc/migracao_ubuntu26/CHANGELOG.md` — **46 itens**, cada alteração e o porquê
- `doc/migracao_ubuntu26/AUDIT_DEPENDENCIAS.md` — dependências e pacotes no 26.04
- `doc/migracao_ubuntu26/CHECKLIST_COMMIT.md` — o que sobe / o que não sobe no commit
- `doc/migracao_ubuntu26/ESTADO_ATUAL.md` — este arquivo
- `doc/migracao_ubuntu26/install/00` a `05` — roteiro de instalação numa máquina nova
- `src/global/opencv_c_compat.h` — header novo de compatibilidade com a API C do OpenCV
- `~/.bashrc` — bloco `#CARMEN` (backup do original em `~/.bashrc.bak.*`)
