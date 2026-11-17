# Deadlock na inversão de marcha (cusp) do `behavior_selector`

**Arquivo alterado:** `src/behavior_selector/behavior_selector_set_goal_list.cpp` (função
`set_goal_list()`, ramo do `SWITCH_VELOCITY_SIGNAL_GOAL`, ~linha 948).
**Data:** 2026-08-28. **Branch:** `ubuntu26`.
**Encontrado em:** manobra de estacionamento de ré (`park at`) de um veículo com ré
habilitada, no simulador.

> A configuração usada no diagnóstico (mapa, grafo, `.ini` e missões de um veículo específico)
> mora fora deste repositório; o defeito e a correção, porém, não dependem dela — valem para
> qualquer veículo com `behavior_selector_reverse_driving on`.

---

## Sintoma

Todo plano que contém um **cusp** (ponto de inversão de marcha: o carro anda para a frente,
para, e segue de ré — ou o contrário) **parava no primeiro cusp e não retomava**. O carro ficava
parado indefinidamente. Planos sem cusp completavam sempre.

Medido: o carro parou a **0,04 m** do ponto marcado como "CUSP 1" no plano do `offroad_planner`.

Como todo estacionamento em vaga fechada exige pelo menos uma inversão de marcha, na prática
**nenhum `park at` com ré chegava ao destino**.

## Diagnóstico

A cadeia foi lida no código e confirmada observando o campo `low_level_state` da mensagem
`behavior_selector_state` por IPC:

1. O `behavior_selector` **detecta o cusp corretamente** e entra na máquina de ré:
   `Stopping_To_Reverse` → `Stopped_At_Reverse_S0` → `Stopped_At_Reverse_S1` →
   **`Stopped_At_Reverse_S2`**. Até aqui, tudo certo.

2. Para **sair** do `Stopped_At_Reverse_S2`
   (`behavior_selector_state_machine.cpp:1234`) o carro precisa satisfazer uma de duas
   condições:

   ```c
   case Stopped_At_Reverse_S2:
       if (autonomous && ((current_robot_pose_v_and_phi.v < -0.5) ||
           (distance_to_reverse_waypoint(current_robot_pose_v_and_phi) >
            (1.0 * (robot_config.distance_between_front_and_rear_axles / 2.625)))))
           decision_making_state_msg->low_level_state = Free_Reverse_Running;
   ```

   ou seja: **já estar andando de ré** a `v < -0,5 m/s`, **ou** estar a mais de
   `entre_eixos / 2,625` metros do waypoint de inversão — para um veículo com entre-eixos
   de 2,485 m, isso dá **0,947 m**.

   Mas o carro está **parado em cima do waypoint**: `v = 0` e distância ≈ 0. Nenhuma das duas
   vale. Ele fica no S2.

3. Quem faria o carro andar é o goal do tipo `SWITCH_VELOCITY_SIGNAL_GOAL`, criado em
   `behavior_selector_set_goal_list.cpp:985`. O ramo que o cria, porém, estava assim (original):

   ```c
   ((next_pose_change_direction_index == rddf_pose_index) && (next_pose_change_direction_index > 0) &&
     (
      (behavior_selector_state_message.low_level_state == Free_Reverse_Running) ||
      (behavior_selector_state_message.low_level_state == Free_Running) ||
      (behavior_selector_state_message.low_level_state == Stopped_At_Reverse_S2) ||
      (behavior_selector_state_message.low_level_state == Stopped_At_Go_Forward_S2)
     )
   )
   ```

   O guarda **`next_pose_change_direction_index > 0`** vale para os quatro estados. Com o carro
   parado exatamente sobre o cusp, o cusp é a **pose 0** do caminho corrente, então
   `next_pose_change_direction_index == 0` e o `> 0` **exclui justamente o caso em que o goal é
   necessário**.

4. **Deadlock:** sem goal o carro não anda; sem andar, o S2 não sai; sem sair do S2, o carro
   nunca volta a `Free_Reverse_Running`.

### Por que o guarda `> 0` existe

Ele não é gratuito. Nos estados de rodagem livre (`Free_Running` / `Free_Reverse_Running`) um
goal no índice 0 significaria colocar um goal **em cima da própria pose atual do carro**, o que
o faria frear sem motivo. Para esses dois estados o `> 0` está correto e foi mantido.

Nos estados `..._S2` a semântica é **oposta**: o carro já está parado sobre o ponto de inversão,
e é exatamente esse goal que o faz arrancar de novo, com o sinal de velocidade trocado. Aqui o
índice 0 é o caso normal, não a anomalia.

## A correção

Os ramos foram **separados**: o guarda `> 0` continua valendo só para `Free_Running` e
`Free_Reverse_Running`; os estados `Stopped_At_Reverse_S2` e `Stopped_At_Go_Forward_S2` passaram
a ter um ramo próprio, sem o guarda.

```diff
 					((next_pose_change_direction_index == rddf_pose_index) && (next_pose_change_direction_index > 0) &&
 					  (
 					   (behavior_selector_state_message.low_level_state == Free_Reverse_Running) ||
-					   (behavior_selector_state_message.low_level_state == Free_Running) ||
+					   (behavior_selector_state_message.low_level_state == Free_Running)
+					  )
+					) ||
+					((next_pose_change_direction_index == rddf_pose_index) &&
+					  (
 					   (behavior_selector_state_message.low_level_state == Stopped_At_Reverse_S2) ||
 					   (behavior_selector_state_message.low_level_state == Stopped_At_Go_Forward_S2)
 					  )
 					)
```

O bloco novo vem comentado no próprio código, junto à alteração.

### O que **não** muda

- O ramo anterior (`Stopping_To_Reverse`, `..._S0`, `..._S1`, e os equivalentes de
  `Go_Forward`) está intacto.
- `Free_Running` e `Free_Reverse_Running` continuam com o mesmo comportamento de antes — o
  guarda `> 0` foi preservado para eles.
- Tudo continua dentro do `if (behavior_selector_reverse_driving && ...)`: com ré desabilitada,
  nada disso é alcançado.
- Nenhuma mudança na máquina de estados (`behavior_selector_state_machine.cpp` **não** foi
  tocado).

## Validação

Testado no simulador, com uma missão de `park at` numa vaga que só se alcança de ré (plano do
`offroad_planner` com 1 cusp: 21 poses para a frente, 43 de ré):

- **Antes:** o carro parava a 0,04 m do primeiro cusp e ficava lá para sempre.
- **Depois:** o carro **passa o cusp pela primeira vez**, inverte a marcha e segue de ré ao
  longo do plano.

Planos sem cusp (medidos antes e depois) continuam completando normalmente — por exemplo
`park at place` pelo nó 21, 129 poses, 0 cusp, chegando a **0,31 m / 3,6°** do alvo em 56 s.

## Pendências relacionadas (não corrigidas por esta alteração)

1. **O carro ainda não termina a manobra de ré.** Depois de passar o cusp, ele encalha contra
   uma parede ~11 m antes da vaga. Na pose onde para, a silhueta do carro com a margem de
   0,30 m do `model_predictive_planner_obstacles_safe_distance` toca **1 célula** de obstáculo;
   com margem 0, nenhuma. Ou seja, o `offroad_planner` aceitou um caminho que o MPP recusa —
   o planejador testa colisão com `trajectory_pose_hit_obstacle` sobre o
   `obstacle_distance_grid_map`, enquanto o MPP usa os círculos do `robot_collision_file`.
   Isto é um problema **diferente**, a jusante desta correção.

2. **O mesmo defeito existe no fork mais novo deste mesmo código**, idêntico linha a linha.

## Armadilha ao depurar isto

Ao decodificar o `low_level_state` recebido por IPC, **ancore a leitura do
`carmen_behavior_selector_low_level_state_t` pelo nome do typedef**, não pelo primeiro
`typedef enum` do header. O enum tem 45 entradas; um decodificador que pegou o enum anterior do
arquivo junto deslocou todos os nomes e me fez ler `Stopped_At_Red_Traffic_Light_S1` no lugar de
`Stopped_At_Reverse_S2` — um patch foi feito no estado errado e depois revertido.

## Referências no código

| O quê | Onde |
|---|---|
| Ramo do `SWITCH_VELOCITY_SIGNAL_GOAL` (alterado) | `behavior_selector_set_goal_list.cpp:948-987` |
| Saída do `Stopped_At_Reverse_S2` | `behavior_selector_state_machine.cpp:1234` |
| Saída do `Stopped_At_Go_Forward_S2` | `behavior_selector_state_machine.cpp:1286` |
