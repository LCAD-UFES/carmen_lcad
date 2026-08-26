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

---

# Segundo bug: o `route_planner` segfaultava com 2+ anotações

Encontrado em 2026-08-26, depois que a correção acima entrou. É **independente** dela e
muito mais antigo.

## Sintoma

`proccontrol` reportando, de forma intermitente:

```
./route_planner ... exited due to SIGNAL (code = 11)
```

Código 11 = `SIGSEGV`. O `proccontrol` respawnava o módulo, mas a instância nova voltava em
`IDLE` sem destino — `set_destination_flag` só é ligado por uma
`carmen_route_planner_destination_message` que chega de fora, e ninguém reenvia. Resultado
prático: o robô terminava o rddf que os módulos de baixo ainda tinham em mãos, parava no
último ponto dele, e nada mais era atualizado.

## Causa raiz: a struct cresceu, o formato IPC não

`carmen_annotation_t` (`src/global/global.h`) tinha **9 campos**:

```c
carmen_vector_3D_t annotation_point;   //  0..24
double annotation_orientation;         // 24..32
char  *annotation_description;         // 32..40
int    annotation_type;                // 40..44
int    annotation_code;                // 44..48
int    annotation_id;                  // 48..52   <-- sem uso
int    size;                           // 52..56   <-- sem uso
double *x_points;                      // 56..64   <-- sem uso
double *y_points;                      // 64..72   <-- sem uso
```

→ `sizeof(carmen_annotation_t)` = **72 bytes**.

Mas o formato IPC em `rddf_messages.h` descreve só os **5 primeiros**:

```
CARMEN_RDDF_ANNOTATION_MESSAGE_FMT
  "{int, <{{double,double,double},double,string,int,int}:1>,double,string}"
```

→ o marshaller anda pelo array de **48 em 48 bytes**.

A partir do **segundo** elemento tudo desalinha. No offset 32 do "elemento 1" (48+32 = 80),
onde o formato espera o `char *annotation_description`, o que está de fato na memória é o
campo `annotation_point.y` do elemento real — um `double`. O IPC chama `strlen()` nesse valor
e morre.

Isso foi confirmado bit a bit no gdb: o ponteiro que estourou era `0x408f412597531754`, que
como IEEE754 vale `1000.1433550349707` — exatamente o `y` da segunda anotação.

```
#0  __strlen_avx2 ()
#1  x_ipcStrLen (s=0x408f412597531754 <Cannot access memory>)
#2  x_ipc_STR_Trans_ELength (dstart=32)
#3  x_ipc_bufferSize1 (dataStruct=<annotation_queue_message>)
...
#9  IPC_publishData (msgName="carmen_rddf_annotation_message")
#10 carmen_rddf_play_publish_annotation_queue ()
#11 build_and_send_rddf_and_annotations ()
#12 route_planner (globalpos)
```

**Com uma anotação só nunca quebra** — apenas o elemento 0 é lido, e os 48 primeiros bytes
dele estão corretos. É exatamente por isso que um mapa funcionava e o outro não:

| Mapa | anotações | comportamento |
|---|---|---|
| `lcad2` | 1 | nunca caiu |
| `ufes_argos` | 6 | caía |
| `argos-rddf-ct13` (simulador) | várias | caía |

Não é regressão da migração: os 4 campos entraram no commit `831a94b58`
("subindo arquivos src/global") sem que o `_FMT` fosse atualizado junto.

## Correção

Os 4 campos **não são usados em lugar nenhum da árvore** (verificado por busca em
`src/` e `sharedlib/`). Foram removidos, e `sizeof(carmen_annotation_t)` voltou a 48 —
casando com o formato.

Optou-se por encolher a struct em vez de estender o `_FMT` porque o formato de 5 campos já
está **compilado dentro dos binários pré-compilados** de `bin/` (`task_manager`,
`offroad_planner`...). Mudar o formato quebraria todos eles de uma vez — exatamente a classe
de problema descrita na primeira metade deste documento.

## A armadilha do rebuild

Mexer em `global.h` obriga a recompilar **todo módulo que usa a struct**, e o
`Makefile.depend` de vários deles **não lista `global.h`** — o `make` os considera em dia e
não os reconstrói. Depois do primeiro rebuild o crash apenas *mudou de lugar*, para dentro da
`librddf_util`, porque `src/rddf/rddf_play_annotations.o` continuava sendo o objeto de 18 de
agosto, ainda com o layout de 72 bytes.

Foi preciso apagar os `.o` à mão:

```bash
for m in global rddf behavior_selector navigator_gui2 viewer_3D; do
    rm -f src/$m/*.o src/$m/*/*.o
    (cd src/$m && make)
done
# e depois os que ficam fora do PACKAGES
for m in route_planner offroad_planner; do rm -f src/$m/*.o; (cd src/$m && make); done
```

Eram **57 objetos velhos**. Como diagnosticar: procure na instrução que falhou um offset
maior que o `sizeof` atual da struct, ou a constante mágica de divisão do `vector::size()`
(`0x6666666666666667` corresponde a elementos de 80 bytes) — ela denuncia código compilado
contra o layout antigo.

## Verificação

Reproduzido e corrigido com o simulador, sem hardware, usando
`data/argos/process/process-argos-navigate.ini`:

| | antes | depois |
|---|---|---|
| SIGSEGV em 70 s | 177 | **0** |
| reinícios do `route_planner` | 178 | **1** (só o inicial) |
| erros de unmarshall | 0 | 0 |

Confirmado ainda com uma rodada de **120 s** com os binários de produção (sem `-g`, sem gdb):
nenhum módulo morreu.

---

# Terceiro bug: "anda ate' um ponto e para"

Encontrado em 2026-08-26, depois dos dois anteriores. Nao e' crash: o robo navega, para no
meio da rota e nao segue mais.

## Causa raiz: 3 velocidades de ruido em 6291 pontos

O rddf gravado com a odometria do Go2 tem velocidades que oscilam em torno de zero quando o
robo esta' praticamente parado. No `rddf_ufes.txt`:

```
pontos                      : 6291
com |v| > 0.05 m/s          : 6287, TODOS positivos
negativos                   : 3   (o mais negativo: -0.035 m/s)
trocas de sinal de v        : indices 0, 115 e 117
```

Nao existe manobra de re' nenhuma nesse percurso. Mas dois consumidores leem o **sinal** de
`v` e concluem que existe:

**1. `save_path_beyond_first_velocity_switch()` corta a rota.** Ela varre o indice do rddf e
quebra no primeiro `carmen_sign(v[i]) != carmen_sign(v[i+1])`, guardando o resto em
`saved_path` para depois. Com o ruido, o corte caiu no ponto 115: dos 597 nos planejados, o
indice ficou com **109**. Media pelo DBG:

```
[DBG] build_route: route_indexes=597
[DBG-rddf] pos=0 size=109 ahead=100
```

A janela de poses entao drenava (100, 97, 92, ... 4, 3) conforme o robo avanccava, ate'
`End_Of_Path_Reached` ~7 m depois -- de uma rota de 37 m.

**2. O `behavior_selector` decide dar re'.** Ele le' o sinal das poses publicadas; ao ver os
pontos negativos a' frente vai para `Stopping_To_Reverse` -> `Free_Reverse_Running`. Os pontos
de ruido ficam em (7757343.74, -363787.13), exatamente onde o robo parava.

## Correção

Duas mudancas em `route_planner_main.cpp`, ambas com o mesmo limiar:

```c
#define MIN_V_FOR_DIRECTION_SWITCH 0.05   // m/s
```

- `save_path_beyond_first_velocity_switch()` passou a ignorar pontos abaixo do limiar ao
  procurar a troca de sentido -- um ponto "parado" nao inicia nem encerra uma manobra.
- `build_route_in_rddf_play_format()` (as duas sobrecargas) sanea o SINAL da velocidade ao
  montar o indice: mantem o modulo e adota o ultimo sentido significativo. Trechos de re' de
  verdade (|v| acima do limiar) continuam sendo respeitados.

Corrigir no planner, e nao no dado, resolve para qualquer mapa -- inclusive rddfs ja'
gravados -- e dispensa regerar grafo.

## Verificação

Simulacao completa LCAD -> CT13 (37 m) com `process-argos-navigate-ufes.ini`:

| | antes | depois |
|---|---|---|
| indice do rddf | 109 de 597 | **597 de 597** |
| janela de poses | drenava 100 -> 3 | **fica em 100** |
| distancia percorrida | ~7 m, parava | **37 m, chegou** |
| estados do behavior_selector | `Stopping_To_Reverse`, `Free_Reverse_Running` | `Free_Running` ate' `End_Of_Path_Reached` |
| pose final | (7757343.7, -363787.2) | (7757361.20, -363791.19) |
| alvo RDDF_PLACE_CT13 | | (7757361.09, -363791.25) -- **12 cm** |

Ao chegar, o `route_planner` volta para `IDLE`, que e' o estado terminal correto.
