# Checklist de commit — migração Ubuntu 26.04

Levantamento do `git status`/`git diff` do working tree (branch **`ubuntu26`**),
organizado para decidir o que vai para o commit. O *porquê* de cada alteração está no
`CHANGELOG.md`; aqui é só o "o que sobe / o que não sobe".

> **Nada foi commitado nem pushed.** Rodar `git diff` antes de qualquer `git add`.

## 1. Resumo

Rode para ver os números atuais:

```bash
cd $CARMEN_HOME
git diff --shortstat                    # arquivos rastreados modificados
git status --short | grep '^??'         # arquivos novos (untracked)
```

Estado no fim do build completo — **565 arquivos rastreados alterados**
(`+2566 / -1346`) e 64 arquivos novos (`??`):

| Tipo | Qtd | O que mudou |
|---|---|---|
| `Makefile` | 238 | flags de Boost/OpenCV/PCL/VTK/Python, `-std=`, `-I` faltando, detecção de SDKs |
| `.c`/`.cpp` | 215 | 90% é troca de `#include` do OpenCV; ~20 arquivos com porte de código real |
| `.h`/`.hpp` | 110 | idem, mais 5 correções de tipo (K&R, `gzFile`, `extern`, `typedef enum`) |
| outros | 2 | `ros1_sc_lio_sam/.../CMakeLists.txt` e `vehicle_tracker/vehicle_tracker.pro` |

Os arquivos com **porte de código real** (não só include) são os que valem uma leitura atenta
na revisão:

```bash
git diff --numstat | awk '$3 ~ /\.(c|cpp|h|hpp)$/ && $1+$2 > 12 {print $3}'
```

## 2. Arquivos rastreados modificados — devem subir

Todos os arquivos modificados são alteração de migração (nenhuma mudança de lógica de
negócio). `git add -u` cobre todos de uma vez.

⚠️ **Antes disso, confira dois pontos que são fáceis de estragar em varredura automática:**

1. **Fim de linha (CRLF)** — alguns arquivos de terceiros vendorizados (`tracker_opentld/
   3rdparty/cvblobs`, `visual_tracker/camshift`, `libstereo`) são CRLF. Se um diff nesses
   arquivos mostrar o arquivo inteiro alterado, o fim de linha foi trocado:
   ```bash
   git diff --numstat | awk '$1+$2 > 50 {print $3}'   # nenhum arquivo deve aparecer aqui
   ```
2. **Encoding (Latin-1)** — alguns desses mesmos arquivos têm comentários acentuados em
   Latin-1 (catalão/espanhol). Uma edição em UTF-8 apaga esses bytes silenciosamente:
   ```bash
   for f in $(git diff --name-only); do
       git show HEAD:$f | iconv -f utf-8 -t utf-8 >/dev/null 2>&1 || echo "não-UTF8: $f"
   done
   ```
   Para cada arquivo listado, conferir no `git diff` que só a linha pretendida mudou.

## 3. Arquivos novos — devem subir

**Documentação da migração** (esta pasta inteira):
```
doc/migracao_ubuntu26/README.md
doc/migracao_ubuntu26/CHANGELOG.md
doc/migracao_ubuntu26/AUDIT_DEPENDENCIAS.md
doc/migracao_ubuntu26/CHECKLIST_COMMIT.md          (este arquivo)
doc/migracao_ubuntu26/install/00_ambiente_e_bashrc.md
doc/migracao_ubuntu26/install/01_ferramentas_de_build.md
doc/migracao_ubuntu26/install/02_boost.md
doc/migracao_ubuntu26/install/03_opencv_pcl_g2o_dlib_eigen.md
doc/migracao_ubuntu26/install/04_bibliotecas_sem_pacote_apt.md
doc/migracao_ubuntu26/install/05_pendencias_conhecidas.md
```

**Header novo de compatibilidade** (ver `CHANGELOG.md`, item 34):
```
src/global/opencv_c_compat.h
```

## 4. NÃO deve subir (artefato local de build)

O `.gitignore` do repo **não** ignora `bin/` e `lib/` em bloco — ele lista os artefatos um a um
(são ~700 linhas). Por isso todo binário novo gerado pelo `make` aparece como `??`, tanto
dentro de `bin/` quanto dentro do diretório do próprio módulo.

Lista atual (build completo, 29 arquivos) — conferida com `file`, todos são
`ELF ... executable`:

```
bin/camera_check
bin/camera_drivers
bin/camera_viewer
src/camera_drivers/camera_check
src/camera_drivers/camera_drivers
src/camera_drivers/camera_viewer
src/can_dump/can_playback
src/can_dump/can_playback_one_can
src/can_dump/can_view
src/can_dump/toyota_corolla_simulator
src/ford_escape_hybrid/fake_odometry
src/ford_escape_hybrid/ford_escape_hybrid_tune_pid_automatic
src/gps/correct_hdt_message
src/joystick_vehicle/joystick_vehicle
src/logger/log_fix_messages
src/logger/print_binary_log
src/mapeditor2/carmen_read
src/mapeditor2/carmen_save
src/mapper/test_diff_map
src/navigator_gui2/2D_map_view
src/rddf/rddf_build_from_file
src/rddf/rddf_resample
src/rddf/test_update_annotation
src/utilities/list_ipc_message/list_ipc_message
src/utilities/print_ipc_message/print_ipc_message
src/utilities/publish_pose/util_publish_dummy_gps
src/utilities/publish_pose/util_publish_final_goal
src/utilities/publish_pose/util_publish_gps_from_globalpos
src/viewer_3D/3D_map_view
```

Regenerar a lista a qualquer momento:

```bash
git status --porcelain | grep '^??' | awk '{print $2}' | \
  while read f; do [ -f "$f" ] && file -b "$f" | grep -q ELF && echo "$f"; done
```

Sugestão: acrescentar essas mesmas linhas ao `.gitignore`, seguindo a convenção que o arquivo
já usa (um caminho por linha), para que não reapareçam a cada `make`.

Também **não versionar**:
- `src/Makefile.vars` — gerado pelo `configure` (já está no `.gitignore`).
- `src/laser` — symlink para `sensors/laser_new` criado pelo `configure`.
- `*/Makefile.depend` — gerados pelo `make depend`.
- `lib/compat/` — symlinks de soname para os binários pré-compilados do 20.04. É contorno
  local de máquina; a receita para recriá-lo está versionada em
  `install/05_pendencias_conhecidas.md`, seção i), e num `README.md` dentro da própria pasta.

## 5. Confirmar com o Miguel antes de subir

- `data/gui/navigator_gui2_annotation.glade` — arquivo novo de GUI que apareceu como `??`;
  não tem relação com a migração (não é citado em nenhum item do `CHANGELOG.md`). Provavelmente
  é trabalho à parte — decidir se entra neste commit ou em outro.
- `bin/argos/process-argos-navigate.ini` — **arquivo de configuração de robô**, não código de
  migração. Alterado para desligar o `proccontrol_gui` (`requested_state` 1 → 0), que não
  compila no 26.04 e respawnava em loop com `exit 127`; ver `CHANGELOG.md`, item 43. Como o
  mesmo problema existe em ~1030 referências nos outros `process-*.ini`, decidir se a correção
  deve ser feita em varredura nos demais `.ini` ou se este commit fica só com o do argos.

## 6. Comandos sugeridos

```bash
cd $CARMEN_HOME
git status                 # confira que está na branch ubuntu26

# 1) tudo que já é rastreado e mudou por causa da migração
git add -u

# 2) doc da migração + header novo
git add doc/migracao_ubuntu26/ src/global/opencv_c_compat.h

# 3) confira que nenhum binário entrou
git diff --cached --name-only | xargs -I{} sh -c 'file -b "{}" | grep -q ELF && echo "BINÁRIO: {}"'

git commit -m "migração Ubuntu 26.04: Boost/OpenCV4/PCL/VTK/Python3/GCC15 + docs da migração"
```

A branch `ubuntu26` não é protegida — dá para
commitar/push direto nela, sem PR para `master`.
