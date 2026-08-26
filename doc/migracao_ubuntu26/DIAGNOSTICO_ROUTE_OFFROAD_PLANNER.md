# Por que o `route_planner` e o `offroad_planner` não estavam funcionando

Diagnóstico de 2026-08-25. Branch `ubuntu26`.

## Sintoma

O robô executava a pré-rota (o trecho que liga a pose atual ao início da rota no grafo),
chegava ao fim dela e **a rota nunca era atualizada**. Ele simplesmente não passava a seguir
o grafo.

## Causa raiz

Incompatibilidade de formato IPC na mensagem `carmen_offroad_planner_plan_message`, entre
os dois binários pré-compilados:

```
bin/offroad_planner  publica: {..., {goal_pose}, double,         string}
bin/route_planner    espera:  {..., {goal_pose}, double, double, string}
                                                         ^^^^^^
```

O `double` a mais é o campo **`time_to_plan`**, em
`include/carmen/offroad_planner_messages.h`. O IPC não conseguia desempacotar a mensagem e
cuspia, a cada plano publicado:

```
IPC_ERROR : Could not unmarshall : message carmen_offroad_planner_plan_message
          : Size of byte array does not match dataSize argument
```

Como a mensagem nunca chegava, `offroad_planner_plan` ficava `NULL` para sempre e a máquina
de estados do `route_planner` **travava em `PLANNING_FROM_POSE_TO_LANE`**, sem nunca avançar
para `EXECUTING_OFFROAD_PLAN` nem para `PUBLISHING_ROUTE`.

### Onde isso trava, no código

`src/route_planner/route_planner_main.cpp`, `case PLANNING_FROM_POSE_TO_LANE`: o avanço de
estado depende de `offroad_planner_plan` ser não-nulo. Sem plano, o `else` só republica o
rddf e o estado se repete indefinidamente.

O fluxo normal seria:

```
COMPUTING_ROUTE
   └─ far_enough_for_offroad_planner_request()?   (>5,5 m ou >30°)
        ├─ não → PUBLISHING_ROUTE
        └─ sim → PLANNING_FROM_POSE_TO_LANE   ← travava aqui
                    └─ plano chega → EXECUTING_OFFROAD_PLAN
                            └─ !within_offroad_plan() → PUBLISHING_ROUTE
```

## Por que aconteceu — cronologia

| Data | Commit | O quê |
|---|---|---|
| 2026-07-04 | `0dc12fcf1` | sobe `bin/route_planner`, **já com** `time_to_plan` (veio de um repositório separado, fora do `carmen_lcad`) |
| 2026-07-05 | `20fc8786c` | adiciona `time_to_plan` ao header em `include/carmen/` |
| 2026-07-17 | `737c8ecb0` | sobe `bin/offroad_planner`, **compilado antes** do campo existir |

O binário do `offroad_planner` foi commitado *depois* da mudança do header, mas foi
*construído* antes dela. Ficou desatualizado em relação ao header desde julho, e ninguém
percebeu porque nenhum dos dois tem fonte no repositório para o `make` recompilar.

**Nada disso veio da migração para o Ubuntu 26.04.** Comparando `master` → `ubuntu26`, os
cinco módulos da hierarquia de controle (`route_planner`, `offroad_planner`,
`behavior_selector`, `model_predictive_planner`, `obstacle_avoider`) só receberam ajustes de
`Makefile` e uma correção de `-fno-common` em `collision_detection.c` — nenhuma mudança de
lógica de planejamento. Os dois binários são byte a byte idênticos nas duas branches.

## Por que demorou para aparecer

Três coisas empilhadas escondiam o erro:

1. **Os dois são binários pré-compilados** commitados em `bin/`, sem fonte no `src/`. Não há
   `make` que os reconstrua, então um desalinhamento entre binário e header não quebra o
   build — só o runtime.
2. **O `proccontrol` não mostra a saída dos módulos.** O erro só aparece se a linha do
   módulo no `.ini` redirecionar `stdout`/`stderr` para um arquivo.
3. **O redirecionamento usava `>` em vez de `>>`.** O `proccontrol` respawna o módulo quando
   ele morre, e o `>` trunca o arquivo — a saída da instância que estourou era apagada pela
   que nascia em seguida.

## Correção aplicada

Escolhida a alternativa de **alinhar o header ao binário que não pode ser recompilado**, já
que não existe fonte do `offroad_planner` no repositório.

| Arquivo | Alteração |
|---|---|
| `include/carmen/offroad_planner_messages.h` | removido `time_to_plan` do struct e do `_FMT` |
| `src/nlp_mat_planner/offroad_planner_messages.h` | idem (é a cópia que o módulo exporta) |
| `src/nlp_mat_planner/nlp_mat_planner_main.cpp` | era o único a escrever no campo; virou `(void) time_to_plan` |
| `lib/liboffroad_planner_interface.a` | recompilada — é ela que carrega a string `_FMT` |
| `bin/route_planner` | relinkado contra a lib nova e reinstalado |

Ninguém **lia** `time_to_plan`; ele só era escrito pelo `nlp_mat_planner`. A remoção é
segura.

> ⚠️ **Não reintroduzir `time_to_plan`** sem antes recompilar o `bin/offroad_planner`. O
> comentário no header registra isso.

### Alternativas descartadas

- **Usar o `nlp_mat_planner` no lugar do `offroad_planner`** — tem fonte, atende todos os
  `PLAN_FROM_*` e já usava o formato novo, mas é um planner diferente: mudaria comportamento.
- **Pedir um `offroad_planner` recompilado contra o header atual** — é a solução correta a
  longo prazo, mas depende de terceiros.

## Verificações feitas

- A string de formato IPC do `bin/route_planner` novo é **byte a byte igual** à do
  `bin/offroad_planner`.
- O layout do struct em C bate campo a campo com o formato: 216 bytes, 8 campos, offsets
  conferidos (`0, 4, 8, 16, 24, 112, 200, 208`).
- `central` + `param_daemon` + `offroad_planner` + `route_planner` sobem juntos sem erro de
  `IPC_defineMsg` nem de unmarshall.
- A `.a` recompilada tem o mesmo objeto e os mesmos símbolos exportados da original.

## Efeito colateral: o port do `route_planner` para o Ubuntu 26.04

Durante o diagnóstico foi descoberto que o port do `route_planner` havia sido **revertido**
por um `git checkout` dentro de `src/route_planner/`. Refeito:

| Arquivo | Correção |
|---|---|
| `src/route_planner/Makefile` | `pkg-config opencv4`; Python via `python3-config`/`python3-embed` em vez de 3.5/3.6/3.8 fixos |
| `route_planner_utils.h`, `road_network_generator_utils.h` | `CV_MAJOR_VERSION >= 3` — o `== 3` caía no `#else`, que inclui `opencv/cv.h`, removido no OpenCV 4 |
| `route_planner_messages.h` + `src/global/global.h` | guard `CARMEN_EDGE_T_DEFINED` no `edge_t` — os dois definem o mesmo typedef anônimo, e o compilador os trata como tipos distintos (`conflicting types for 'edge_t'`) |

Consequência: `bin/route_planner` agora linka `libpython3.14` diretamente e **não depende
mais** do symlink de compatibilidade em `lib/compat/libpython3.8.so.1.0`. Em compensação, o
binário passou a ser específico do Ubuntu 26.04.

`src/route_planner/` está no `.gitignore` (o fonte vem de um repositório separado), então só
o binário e os headers públicos entram no commit.

## Armadilha remanescente — a janela de lookahead

Não é bug, mas afeta o teste da pré-rota. `within_offroad_plan()` só libera a troca para
`PUBLISHING_ROUTE` quando o ponto de junção sai da **janela inteira** de
`rddf_num_poses_ahead` — não no instante em que o robô encosta na lane. O tamanho dessa
janela é `rddf_num_poses_ahead × rddf_min_distance_between_waypoints`:

| Mapa | espaçamento | janela |
|---|---|---|
| `carmen-argos.ini` (100 poses) | 0,05 m configurado | 5 m |
| rddf da UFES gravado em 2026-08-25 | 0,070 m medido | ~7 m |

O comentário em `carmen-argos.ini:1037` diz que o
`behavior_selector_rddf_num_poses_ahead_limit = 150` foi dimensionado supondo espaçamento de
**0,5 m** (75 m de janela). O valor atual está ~10× mais denso do que o resto dos parâmetros
supõe. Numa rota de 442 m, 7 m de lookahead é pouco.
