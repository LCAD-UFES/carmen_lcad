# carmen_server — protocolo ZMQ

Módulo carmen que roda **no robô**, junto com os outros, e faz a ponte entre o IPC e um app
remoto (`src/carmen_app`, C++/Qt, roda no notebook sem carmen). Não exige mudança em nenhum outro módulo: só assina mensagens que
já existem e publica comandos que já existem. Também substitui o `proccontrol_gui` — no process,
a linha do `proc_control` roda `./carmen_server` no lugar de `./proccontrol_gui`.

Build (fora do `PACKAGES`): `make -C src/carmen_server`. Depende de `libzmq` e `zlib`.

```
./carmen_server [-pub_port 5560] [-cmd_port 5561] [-pose_hz 10] [-path_hz 2] [-plan_hz 5]
                [-map_period 2] [-max_path_points 400] [-missions_folder PASTA]
                [-annotation_path ARQ] [-map_path PASTA]
```

Tudo roda num thread só, no loop do IPC; um timer de 50 Hz atende os comandos e despacha o que
estiver pendente. O PUB tem fila curta (HWM 20) e nunca bloqueia: se a rede não der conta, as
mensagens são descartadas e o robô segue igual.

## Dados — PUB na porta 5560

Cada mensagem é multipart `[tópico, json]` ou `[tópico, json, binário]`.

| Tópico | Origem (IPC) | Taxa | Conteúdo |
|---|---|---|---|
| `hello` | — | 1 Hz | `{t, host, version}` (para o app saber que está conectado) |
| `pose` | `localize_ackerman_globalpos` | ≤ `-pose_hz` | `{t, x, y, theta, v, phi, converged}` |
| `map` | `map_server_offline_map` | quando muda, ≤ 1/`-map_period` | json `{t, x_origin, y_origin, resolution, x_size, y_size, encoding:"zlib-u8", unknown:255}` + binário |
| `path` | `behavior_selector_path_goals_and_annotations` | ≤ `-path_hz` | `{t, path:[[x,y,theta,v]...], goals:[[x,y,theta,v]...]}` |
| `plan` | `navigator_ackerman_plan` (MPP) | ≤ `-plan_hz` | `{t, path:[[x,y,theta,v]...]}` |
| `route` | `route_planner_road_network` | ≤ 1 Hz | `{t, path:[[x,y,theta,v]...]}` |
| `goal` | `rddf_end_point` | a cada mensagem | `{t, x, y, theta}` |
| `behavior` | `behavior_selector_current_state` | quando muda, ou 1 Hz | `{t, task, state, route_planner_state, task_code, algorithm}` (task/state em nome; task_code e algorithm são os números dos enums do behavior_selector) |
| `proc` | `proccontrol_pidtable` | ≤ 2 Hz | `{t, host, processes:[{group, module, active, requested, pid}]}` |
| `out` | `proccontrol_output` | agrupado a cada 0,2 s | `{module, text}` |
| `lanes` | `route_planner_road_network` (faixas vizinhas) | ≤ 1 Hz | `{t, lanes:[[[x,y,theta,v]...]...]}` — o grafo em volta do carro |
| `frenets` | `frenet_path_planner_set_of_paths` (do behavior_selector) | ≤ `-plan_hz` | `{t, selected, paths:[[...]...], collision:[f...], rddf:[...]}` (40 pontos por caminho; `collision[i]` = fração do comprimento do caminho i onde um círculo de colisão do robô encosta num obstáculo, pelo mapa compacto do obstacle_distance_mapper; -1 = livre) |
| `annotations` | `rddf_annotation_message` | quando muda, ou 0,5 Hz | `{t, items:[{x, y, theta, type, code, text}]}` |
| `offroad` | `offroad_planner_plan` | ≤ 1 Hz | `{t, path:[...]}` (pré-rota) |
| `odom` | `base_ackerman_odometry` | ≤ `-pose_hz` | `{t, v, phi}` |
| `cmd` | `base_ackerman_motion_command` | ≤ 5 Hz | `{t, v, phi}` (primeiro comando) |
| `car` | `ford_escape_status` | ≤ 5 Hz | `{t, throttle, brake, steering_pct, gear, gear_raw, engine, parking_brake, turn_signal, headlights, horn}` |
| `nav` | `navigator_ackerman_status` | ≤ 2 Hz | `{t, autonomous, goal_set, goal_distance}` |
| `mission` | `task_manager_mission_state` | a cada mensagem | `{t, state, id, info}` |
| `robot` | `param_daemon` (robot_*) | uma vez, ~2 s depois de subir | `{length, width, axle, rear_overhang, max_v, max_phi}` |

O `plan_tree` do MPP fica de fora de propósito: é um array fixo de 100×500 pontos (~4 MB por
mensagem), e assiná-lo pesaria no robô. Os frenets vêm da `set_of_paths`, que é variável e leve.

Caminhos com mais de `-max_path_points` pontos são dizimados (o último ponto é sempre mantido).

**Por que o servidor pede o mapa:** o `map_server` só publica o `offline_map` na primeira pose e
quando o robô troca de bloco. O servidor pede o mapa atual ao `map_server` na partida (se ainda não
chegou nenhum), 1 s depois de um `initial_pose` e no `get_map` com `refresh`. O app também pede pelo
`get_map` (canal REQ, que não descarta) quando o robô sai da área do mapa que ele tem.

**Mapa:** o binário é a grade comprimida com zlib, um byte por célula: probabilidade de ocupação
0..1 → 0..254, desconhecido → 255. A ordem é a do carmen, `complete_map[x * y_size + y]`, ou seja,
um array C de forma `(x_size, y_size)` indexado `[x][y]`. A célula `[i][j]` cobre
`x_origin + i*resolution`, `y_origin + j*resolution`.

## Comandos — REP na porta 5561

Pedido: um json com `cmd`. Resposta: `[json]` ou `[json, binário]`; o json sempre tem `ok`
(e `error` quando `ok` é `false`).

| `cmd` | Campos | Faz o mesmo que | Resposta |
|---|---|---|---|
| `ping` | — | — | `{ok, version, host}` |
| `snapshot` | — | — | `{ok, version, host, pose, proc, goal, behavior, car, nav, mission, robot, annotation_file, map}` + binário do mapa (se houver). Para quem acabou de conectar. |
| `initial_pose` | `x, y, theta` | `util_publish_initial_pose` (`localize_ackerman_initialize_gaussian_command`) | `{ok}` |
| `final_goal` | `x, y, theta` | `util_publish_final_goal` / clique do `navigator_gui2` (`rddf_publish_end_point_message(50, ...)`) | `{ok}` |
| `route_to` | `x, y, theta` | botão *Compute Route* do `navigator_gui2` (`route_planner_set_destination`) | `{ok}` |
| `module` | `name, state` (0/1) | `proccontrol_gui` (`proccontrol_set_module_state`) | `{ok}` |
| `group` | `name, state` (0/1) | `proccontrol_gui` (`proccontrol_set_group_state`) | `{ok}` |
| `get_output` | `name` | — | `{ok, module, lines:[...]}` (últimas 300 linhas) |
| `get_map` | `refresh` (0/1) | com `refresh`: pede o mapa atual ao `map_server` (`REQUEST_CURRENT_OFFLINE_MAP`), como o `viewer_3D_dash` | `{ok, map}` + binário |
| `full_map` | `max_side` (padrão 3000) | — | `{ok, map:{..., blocks}}` + binário: mosaico de todos os `m<x>_<y>.map` de `-map_path`, reduzido para caber em `max_side` células (guardado em cache) |
| `car_model` | — | `{ok, model:{file, size:[x,y,z], yaw, bytes, encoding:"zlib-obj"}}` + binário: o `.obj` do carro (parâmetros `carmodel_*`, o mesmo do viewer_3D) comprimido com zlib; `bytes` é o tamanho original. Lido uma vez e guardado em cache |
| `map_layer` | `source` (`none`, `mapper`, `mapper_level1`, `cost`, `likelihood`, `global_likelihood`, `remission`, `moving_objects`, `road`), `period` (s, padrão 1), `max_side` (padrão 600) | `{ok, source}` | Assina SÓ o mapa escolhido (o menu Maps do navigator_gui2) e publica no tópico `map2` (mesmo formato do `map` + `source`), reduzido a `max_side` células (máximo do bloco, para obstáculo não sumir). `likelihood`/`remission` só chegam quando o map_server publica o mapa de localização (troca de bloco). Sem renovar em 30 s, desliga |
| `lidar` | `id` (-1 desliga), `hz` (padrão 2), `max_points` (padrão 15000) | `{ok, id}` | Assina a `variable_scan` do lidar `id` e publica no tópico `lidar` (json `{t,id,n,x,y,theta}` + binário zlib de n × [int16 x,y,z em cm no referencial do carro, u8 intensidade]); `x,y,theta` = último globalpos. Decima disparos/raios até `max_points`. Sem renovar em 30 s, desliga |
| `camera` | `id` (0..9), `on` (0/1), `hz` (padrão 5), `width` (padrão 480), `quality` (padrão 60) | `{ok, id, on}` | Assina a `camera<id>` do camera_drivers e publica no tópico `camera` (json `{t,id,w,h,encoding:"jpeg"}` + JPEG). Reduz por fator inteiro até `width`. Sem renovar em 30 s, desliga |
| `behavior` | `task` (enum do behavior_selector: 0 Follow Route, 1 Park, ...), `algorithm` (0 Frenet, 1 A*, 2 RRT, 3 RDDF, 4 Gradient; omitido = só a tarefa) | `{ok}` | Os combos do navigator_gui2 (`carmen_behavior_selector_set_algorithm` / `_set_task`) |
| `annotations` | — | — | `{ok, items:[...]}`: todas as anotações do arquivo `-annotation_path` (o mesmo do `navigator_gui2`) |
| `go` / `stop` | — | botões GO/STOP do `viewer_3D_dash` (`navigator_ackerman_go/stop`) | `{ok}` |
| `compute_route` | — | *Compute Route* do `navigator_gui2` até o destino marcado (último `rddf_end_point`) | `{ok}` ou erro se não há destino |
| `missions` | — | — | `{ok, folder, missions:[nomes]}` (`.txt` da `-missions_folder`) |
| `execute_mission` | `name` | `viewer_3D_dash` (`user_app_server_publish_execute_mission`, abortando a anterior) | `{ok}` |
| `abort_mission` | — | `user_app_server_publish_update_mission(ABORT)` | `{ok}` |

## Watchdog do app

Com `-app_watchdog N` (padrão 5 s; 0 desliga), se o servidor ficar N s sem receber nenhum pedido do app
depois do primeiro, ele chama `carmen_navigator_ackerman_stop()` (e repete a cada 2 s enquanto o
navigator disser que está autônomo) e publica no tópico `watchdog` `{state:"stopped", silence}`. No
primeiro pedido seguinte publica `{state:"ok"}`; o robô continua parado até um novo `go`. O app manda
`ping` a cada 1 s para isso.

## Testar

No robô: `cd bin && ./proccontrol ../data/argos/process/process-argos-navigate-carmen-server.ini`
(com o `central` rodando). No notebook: `make -C src/carmen_app` e
`src/carmen_app/carmen_app --host <IP do robô>` (ou `~/Desktop/carmen-app.sh --host <IP>`).
