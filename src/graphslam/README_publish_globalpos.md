# graphslam_publish_globalpos

Publica `carmen_localize_ackerman_globalpos_message` **direto** a partir de um arquivo de
poses otimizadas (`poses_opt.dat`), dispensando o `localize_ackerman` no fluxo de geração
de mapa.

O `graphslam_publish` antigo **continua existindo e não foi alterado**. Este é um binário
separado; os dois convivem no mesmo módulo.

---

## Por que existe

O `graphslam_publish` original publica apenas `carmen_fused_odometry_message`. A função
`assembly_and_publish_globalpos_message`, que ele carrega, é código morto — nunca é
chamada. Como o `mapper` só assina `globalpos`, era obrigatório colocar um
`./localize_ackerman -mapping_mode on` no meio do caminho só para converter
`fused_odometry` → `globalpos`.

Isso cria **três casamentos de timestamp encadeados, nenhum deles com tolerância**:

| # | onde | o que faz | limite |
|---|---|---|---|
| 1 | `graphslam_publish` | pega a pose anterior mais próxima | **nenhum** — aceita pose de qualquer idade; e só publica com `v > 0.1`, então com o carro parado fica mudo |
| 2 | `localize_ackerman` | pega a `fused_odometry` mais próxima e extrapola `ds = v·dt` | **nenhum** — há um `TODO` no próprio fonte perguntando se não deveria deixar de publicar quando `dt` é grande |
| 3 | `mapper` | usa sempre a `globalpos` mais recente | **nenhum** — nunca compara o timestamp dela com o do scan |

O resultado é pose velha aplicada a scan novo. No `mapper` isso não apenas pinta no lugar
errado: o atraso entra como offset de *deskew* em

```c
dt1 = points_timestamp - robot_timestamp - N*dt;   // mapper.cpp
```

ou seja, a nuvem é **esticada** proporcionalmente ao atraso da pose. É a assinatura de
borrão ("mancha") no mapa.

### O que este módulo faz diferente

```
ANTES:  poses_opt ─► graphslam_publish ─► fused_odometry ─► localize_ackerman ─► globalpos ─► mapper
                     (casa por odometria,                   (casa por vizinho +
                      sem tolerância)                        extrapola sem teto)

AGORA:  poses_opt ─► graphslam_publish_globalpos ─────────────────────────────► globalpos ─► mapper
                     (casa com o timestamp do PRÓPRIO scan de lidar,
                      exigindo 1 µs; sem casamento, NÃO publica)
```

- casa a pose com o timestamp da **própria mensagem de lidar**;
- exige casamento **exato** (1 µs por padrão);
- sem casamento exato **não publica nada** — é melhor o `mapper` ficar sem pose nova do que
  pintar com pose errada;
- **nenhuma extrapolação**;
- **nenhum gate de velocidade** — com o carro parado continua publicando, desde que exista
  pose para aquele scan.

Essa é a mesma disciplina que um fork deste código já usava.

---

## Uso

```
./graphslam_publish_globalpos <poses_opt.dat> [opções]
```

| opção | padrão | o que faz |
|---|---|---|
| `-poses_from lidar\|velodyne\|odometry` | `lidar` | em que mensagem casar e publicar |
| `-lidar_id N` | `5` | id do *variable scan* a assinar (só em `-poses_from lidar`) |
| `-tolerance S` | `1e-6` | janela de casamento exato, em segundos |
| `-fake_timestamp on\|off` | `off` | `on` aceita a pose mais próxima seja qual for a distância. **Só para depurar** — é exatamente o comportamento frouxo que borra o mapa |
| `-verbose on\|off` | `on` | estatísticas a cada 5 s |
| `-save_globalpos_file <arquivo>` | — | grava cada globalpos publicada: `x  y  theta  v  phi  timestamp` separados por TAB — o **mesmo formato** do `localize_ackerman -save_globalpos_file` |

Uma flag escrita errado é **erro fatal**, não silêncio (o `graphslam_publish` antigo ignora
flags desconhecidas sem avisar).

### Formatos de arquivo aceitos

| colunas | conteúdo |
|---|---|
| 4 | `x y theta t_vertice` |
| 6 | `x y theta t_lidar t_odom t_vertice` (todos os `t` viram `t_vertice`) |
| 7 | `x y theta t_vertice t_lidar t_odom t_gps` |
| 10 | `x y z roll pitch yaw t_vertice t_lidar t_odom t_gps` — pose 6D do SC-LIO-SAM |

Colunas de timestamp valem `-1` quando não existem. O ARGOS não tem odometria Ackermann,
então `t_odom = -1` e o modo correto é `-poses_from lidar` (o padrão).

### `v` e `phi`

Saem da odometria casada **exatamente** com `t_odom` da pose. Sem casamento exato ficam
**zerados de propósito**: `v` e `phi` só alimentam o deskew do `mapper`, e um valor chutado
ali estica a nuvem. Zero significa "não compense movimento", que é o comportamento seguro.
No ARGOS esse é o caminho normal.

### O timestamp precisa ser o do CARMEN

O casamento é contra o timestamp da mensagem de lidar **do log**. Um `poses_opt.dat` gravado
com timestamp de relógio de parede (ou com o `header.stamp` do ROS em vez do timestamp
CARMEN) não casa com nada, e o módulo vai reportar 100% de scans sem pose exata. No
SC-LIO-SAM é o `posesCarmenStampTopic` (`/carmen/scan_time_reference`) que garante isso.

---

## Gate de pose no `mapper` (opcional, desligado por padrão)

O `mapper` sempre usou `globalpos_history[last_globalpos]` — a pose **mais recente** — sem
comparar o timestamp dela com o do scan. Dois gates novos permitem recusar o scan quando a
pose não é confiável. **Ambos vêm desligados; com os valores padrão o comportamento é
exatamente o de antes.**

| parâmetro | padrão | o que faz |
|---|---|---|
| `-pose_gate_max_delay S` | `0` (desligado) | descarta o scan se `\|scan_ts − pose_ts\| > S` |
| `-pose_gate_jump M` | `0` (desligado) | pose que anda mais que `M` metros entre duas mensagens conta como **salto** |
| `-pose_gate_settle N` | `0` | quantos scans descartar depois de cada salto |

O gate de salto existe para o outro caso que mancha o mapa: quando o localizador **reencaixa**
o robô (loop closure, relocalização por Scan Context, clique de pose inicial). Cada reencaixe
é uma descontinuidade na trajetória, e o `mapper` pintava atravessando ela. O SC-LIO-SAM já
faz esse mesmo descarte do lado dele ao gravar o `poses_opt.dat`
(`posesSettleScans: 20`, ~1 s a 20 Hz); estes parâmetros levam a mesma disciplina para o mapa.

Valores razoáveis para o ARGOS (lidar a 20 Hz):

```
./mapper -map_path ... -mapping_mode on -pose_gate_max_delay 0.06 -pose_gate_jump 0.5 -pose_gate_settle 20
```

Os descartes são reportados no stderr do `mapper` (lembre que sob o `proccontrol` isso só
aparece no `proccontrol_viewoutput` ou na janela de output do `proccontrol_gui`).

---

## `.ini` pronto: EE2

`bin/argos/process-argos-EE2-globalpos.ini` roda o log `log_argos_EE_20260817-1.txt` com o
`poses_opt.dat` do EE2, **sem `localize_ackerman`**, com o gate de pose ligado e gravando a
globalpos em `/dados/sc_lio_sam_output/EE2/globalpos_EE2.txt`:

```
./proccontrol argos/process-argos-EE2-globalpos.ini
```

### Reaproveitando um mapeamento antigo: `bin/optimized_poses_to_poses_opt.py`

O EE2 só tinha a saída crua do SC-LIO-SAM (`optimized_poses.txt` + `times.txt`), sem
`poses_opt.dat`. O `times.txt` traz o tempo do *keyframe*, que **não é** bit-a-bit o
timestamp da mensagem de lidar do log (no EE a diferença chega a 50 ms) — e casamento de
1 µs não perdoa isso. O script recarimba cada keyframe no timestamp exato do scan mais
próximo e converte a matriz 3×4 para `x y z roll pitch yaw`:

```
./optimized_poses_to_poses_opt.py /dados/sc_lio_sam_output/EE2 \
    /dados/logs_argos/log_argos_EE_20260817-1.txt \
    /dados/sc_lio_sam_output/EE2/poses_opt.dat \
    --origin-x 7757339.037 --origin-y -363784.117
```

**A cobertura fica limitada aos keyframes** — no EE2, 378 poses para 1583 scans (24%). Os
outros 76% não têm pose exata e não são pintados. Para cobertura cheia, gere o
`poses_opt.dat` rodando o SC-LIO-SAM com a ponte `/carmen/scan_time_reference`
(`writeLocalizationPose`), que já sai com o timestamp CARMEN de cada scan.

---

## Exemplo de `.ini`

Repare que **não há `localize_ackerman`**:

```ini
SET LOG_PATH=/dados/logs_argos/log_argos_EE_20260817-1.txt
SET PARAM_PATH=argos/carmen-argos.ini
SET POSES_PATH=/dados/sc_lio_sam_output/EE2/poses_opt.dat
SET MAP_FILES_PATH=/dados/sc_lio_sam_output/EE2/map2d

# map_x/map_y sao a UTM inicial do log (CARMEN_PARAM_INT), nao um numero qualquer
SET MAP_X=7757339
SET MAP_Y=-363784

 param_daemon     support   1  0  ./param_daemon ${PARAM_PATH}
 playback         support   1  0  ./playback ${LOG_PATH}
 playback_control support   1  0  ./playback_control -message "t 0" -autostart on
 proccontrol_gui  support   1  0  ./proccontrol_gui
 map_server       support   1  0  ./map_server -map_path ${MAP_FILES_PATH} -map_x ${MAP_X} -map_y ${MAP_Y} -block_map on -lanemap_incoming_message_type 0
 base_ackerman    filter    1  0  ./base_ackerman
 mapper           SLAM      1  0  ./mapper -map_path ${MAP_FILES_PATH} -mapping_mode on -pose_gate_max_delay 0.06
 navigator_gui    interface 1  0  ./navigator_gui2 -map_path ${MAP_FILES_PATH}
 pub_poses        graphslam 1  0  ./graphslam_publish_globalpos ${POSES_PATH} -poses_from lidar -lidar_id 5
 complete_map     graphslam 0  0  ./build_complete_map -map_path ${MAP_FILES_PATH}
```

Limpe o `map_path` antes de cada rodada: blocos velhos em coordenadas erradas fazem o
`build_complete_map` tentar um mapa de milhões de metros.

---

## Como saber que funcionou

- o módulo imprime `N publicadas, M sem pose exata (P%), pior casamento X s`. **`pior
  casamento` tem que ser ~0**; se estiver na casa dos milissegundos, os timestamps do
  `poses_opt.dat` não são os do log;
- `100%` de "sem pose exata" = os timestamps não batem com o log (época errada, ou o arquivo
  foi gravado sem a ponte de timestamp CARMEN);
- os nomes dos blocos em `map_path` têm que trazer as coordenadas UTM da trajetória
  (`m7757337_-363790.map`), não algo perto de zero nem do `-map_x/-map_y`;
- os blocos `m*` têm que ter **md5 diferentes entre si**. Todos iguais = blocos em branco
  criados pelo `map_server`, com o `mapper` sem escrever nada.

---

## Arquivos

| arquivo | o que mudou |
|---|---|
| `src/graphslam/graphslam_publish_globalpos_main.cpp` | **novo** |
| `src/graphslam/Makefile` | novo alvo; `graphslam_publish` intacto |
| `src/mapper/mapper.cpp` | gate de pose (desligado por padrão) + 3 parâmetros |
| `bin/argos/process-argos-EE2-globalpos.ini` | **novo** — `.ini` de teste do EE2, sem `localize_ackerman` |
| `bin/optimized_poses_to_poses_opt.py` | **novo** — gera `poses_opt.dat` da saída crua do SC-LIO-SAM |
| `src/grid_mapping/grid_mapping.c` | correção incidental: `full_map_path[100]` → `[1024]` em `carmen_grid_mapping_save_block_map_by_origin`. Um `map_path` com mais de ~80 caracteres estourava a pilha e matava o `mapper` com `*** buffer overflow detected ***`. A função irmã da linha 420 já usava 1024. |

---

## O mapa precisa estar em UTM?

**Não.** Não existe nenhuma referência a UTM no `mapper`, no `localize_ackerman` nem no
`grid_mapping` — um `grep -i utm` nesses três módulos não retorna nada. Eles trabalham em
metros num referencial global qualquer, com `double`. O que dá a impressão de UTM é só a
convenção de quem alimenta a pose (`gps_xyz` produz UTM) e o nome dos blocos, que carrega a
coordenada da origem.

### O que realmente restringe

| item | restrição | ARGOS |
|---|---|---|
| `carmen_grid_mapping_get_map_origin` | usa `floor()` — correto para coordenadas negativas | ok |
| nome do bloco: `sprintf("%s/%c%d_%d.map", ...)` | a origem vira `(int)`. Exato desde que o tamanho do bloco (`mapper_map_width / 3`) seja **inteiro** | `21/3 = 7` m, ok |
| `map_server -map_x -map_y` | são `CARMEN_PARAM_INT` — passe inteiros | `-map_x 0 -map_y 0` funciona |

### Verificado na prática

A mesma trajetória do log `log_argos_EE_20260817-1.txt` foi mapeada duas vezes:

| referencial | `-map_x/-map_y` | blocos | resultado |
|---|---|---|---|
| UTM (`7757339 / -363784`) | `7757339 / -363784` | 18 (grade 6×3) | `m7757337_-363790.map` … |
| local, origem em zero | `0 / 0` | 16 (grade 4×4) | `m-7_-14.map`, `m0_0.map`, … |

Grade **completa nos dois casos, sem buraco**, com conteúdo real (md5 distintos entre os
blocos). A contagem difere só porque o `floor()` quantiza em posições diferentes depois do
deslocamento. Nomes de bloco negativos funcionam normalmente.

### A única armadilha: `x_origin == 0.0` é usado como sentinela

Em três pontos o `mapper` trata origem zero como "origem inválida" e **pula o salvamento**:

```c
// mapper.cpp:1028, mapper.cpp:1200 e mapper.cpp:1924
if (... && offline_map_available && ... && (map_set->occupancy_map->config.x_origin != 0.0))
    carmen_grid_mapping_save_block_map_by_origin(...);
```

(há um caso equivalente em `mapper/neural_mapper_io.cpp:186`.)

Com UTM, `x_origin` fica na casa dos milhões e nunca é zero — a sentinela é invisível. Com
referencial local, `x_origin` passa por zero sempre que o robô estiver na faixa em que
`floor(floor(x/B) - 1) * B == 0`, ou seja `x ∈ [B, 2B)` — para o ARGOS, `x` entre **7 e 14 m**.

**Os três pontos exigem `offline_map_available`**, que só vira `true` quando o `map_server`
publica um *offline map* (`mapper_main.cpp:968`). Por isso o teste acima saiu limpo: mapa
novo, sem offline map, sentinela inalcançável. Ela pode morder quando você **remapeia por
cima de um mapa existente** ou liga `mapper_update_and_merge_with_mapper_saved_maps on`.

### Recomendação

Duas saídas, ambas válidas:

1. **Deslocar a origem para longe de zero** — é exatamente o que o "UTM fake" faz. Não
   precisa ser UTM de verdade: `1000, 1000` já basta para a sentinela nunca ser atingida. É
   a opção sem alterar código.
2. **Corrigir a sentinela**, trocando `x_origin != 0.0` por um flag explícito de "origem já
   inicializada". É a correção certa, mas mexe em caminho de código usado por todo mundo —
   não foi feita aqui.
