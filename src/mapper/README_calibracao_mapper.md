# Calibração do mapper — grade de ocupação

Como o `mapper` decide o que vira obstáculo na grade 2D, quais parâmetros controlam cada
decisão, e como calibrá-los. Os números concretos são os da IARA
(`data/iara/parameters/carmen.ini`, LiDAR Hesai OT-128 de 128 canais).

---

## 1. Quem escreve na grade

```
playback ──scan──> mapper ──> map_files/*.map
                     ▲
                     └── globalpos (pose do veículo)
```

O `mapper` só grava a grade com `-mapping_mode on`. Sem isso ele apenas consome o mapa
offline. A pose vem da `globalpos`; sem ela o mapper não pinta nada.

Cada raio do LiDAR vira uma decisão numa célula. São **três filtros em série**, e um ponto
precisa passar pelos três para virar obstáculo:

1. **Banda de altura** — o ponto está numa faixa de altura que interessa?
2. **Alcance** — a distância medida é válida?
3. **Modelo de ocupação** — a geometria indica obstáculo ou chão?

Calibrar o mapper é ajustar esses três, nessa ordem. Não adianta mexer no terceiro se o
primeiro está deixando passar a copa das árvores.

---

## 2. Banda de altura — "está pegando o teto"

### A regra

`sharedlib/prob_models/prob_map.cpp`, `carmen_prob_models_unaceptable_height_band()`:

```c
if ((obstacle_height > unsafe_height_above_ground) ||
    (obstacle_height < safe_height_from_ground))
        return 1;   // ponto rejeitado: vira log_odds_l0 (neutro), nao obstaculo
```

`obstacle_height` é a altura do ponto **acima do chão**, em metros — já com o raio de roda
somado, já com a pose do veículo aplicada. Não é altura relativa ao sensor.

A banda é **por LiDAR**. Dois sensores no mesmo carro podem ter bandas diferentes: um no
teto vendo longe, outro no para-choque vendo o próximo.

### Os parâmetros

```ini
# global — vale para todo sensor que nao tiver chave propria
mapper_safe_height_from_ground        0.0     # piso da banda, metros acima do chao
mapper_unsafe_height_above_ground     3.0     # teto da banda

# por LiDAR — sobrescreve a global. Chaves OPCIONAIS.
mapper_lidar0_safe_height_from_ground        0.0
mapper_lidar0_unsafe_height_above_ground     2.5
```

O mesmo par existe para o `localize_ackerman`, com o prefixo trocado:

```ini
localize_ackerman_lidar0_safe_height_from_ground     0.0
localize_ackerman_lidar0_unsafe_height_above_ground  2.5
```

**Os dois têm que casar.** Se o localize aceitar obstáculo que o mapa não tem — ou o
contrário — a nuvem não encaixa na grade e a localização degrada.

### Como escolher o teto

O teto é a altura máxima que interessa como obstáculo. A conta é direta:

```
teto = altura do veiculo mais alto que precisa aparecer  +  folga
```

Para a IARA, `2.5 m` cobre carro, van e pedestre com folga, e corta:

- copa de árvore sobre a via
- placa e semáforo em braço projetado
- marquise, viaduto, garagem

Se o mapa está pegando copa de árvore, o teto está alto demais — **ou não está sendo lido**.
Veja a seção 6 (diagnóstico) antes de mexer no valor.

### Como escolher o piso

O piso corta o chão e o ruído rasante. `0.0` significa "tudo o que estiver abaixo do plano
do chão é descartado". Subir para `0.15`–`0.30` ajuda quando o mapa fica sujo de chão em
terreno irregular, mas **some com obstáculo baixo** (guia, tora, buraco). Em via urbana,
`0.0` costuma ser o certo.

### O parâmetro antigo, que não vale mais

```ini
mapper_safe_range_above_sensors    0.5
```

Este era o teto na regra antiga:

```
teto = highest_sensor + safe_range_above_sensors
```

Amarrado a **onde o sensor está montado**. Isso tinha dois problemas: mudar o LiDAR de
altura mudava, em silêncio, o que o mapa considera obstáculo; e não havia como dar bandas
diferentes a dois LiDARs do mesmo carro.

Para referência, com a montagem atual da IARA a regra antiga dava:

```
highest_sensor = sensor_board_1_z 1.482 + lidar0_z 0.35 + robot_wheel_radius 0.28 = 2.112 m
teto antigo    = 2.112 + 0.5 = 2.612 m
```

O parâmetro continua no `.ini` porque outros módulos ainda o leem. **O mapper e o localize
não usam mais.** Mexer nele não muda a grade.

---

## 3. Alcance — "está pegando uma distância ruim"

### Da leitura crua ao metro

```c
range = distance[ray_order[j]] / range_division_factor;
if (range <= 0.0)
    range = range_max;          // <-- ATENCAO
```

```ini
lidar0_range_division_factor   250      # conta bruta -> metros
lidar0_max_range               230.0    # alcance maximo considerado
```

Uma leitura de `12500` vira `12500 / 250 = 50 m`.

### A pegadinha do range zero

O LiDAR devolve `0` quando **não houve retorno** — céu, superfície muito escura, vidro,
chuva. O código trata isso como **alcance máximo**. Ou seja: um raio apontado para o céu
carimba "livre" numa linha reta de **230 metros**.

Em mapeamento isso é o que se quer (limpa o espaço livre), mas com `max_range` grande demais
o mapa fica com rastros de "livre" atravessando quarteirão. Se a sua grade tem faixas
limpas atravessando prédio, é isto.

**Como calibrar:** ponha `lidar0_max_range` no alcance em que o sensor ainda devolve retorno
**confiável**, não no alcance de catálogo. Para o OT-128 em ambiente urbano, algo entre
`80` e `120 m` costuma ser mais honesto que `230`. Meça: rode um trecho e veja a que
distância os pontos começam a ficar esparsos.

### Parâmetros que NÃO cortam o alcance no mapper

```ini
lidar0_min_sensing    150      # 0.6 m  -- lido, mas nao usado pelo mapper
lidar0_max_sensing    24000    # 96 m   -- lido, mas nao usado pelo mapper
```

Estes dois entram no `carmen_lidar_config` e são usados pelo **driver**, não pelo mapper nem
pelo `prob_models`. Conferido varrendo `velodyne_interface.cpp`, `prob_map.cpp` e
`mapper.cpp`: não há corte de raio por eles. Mudar `max_sensing` para 96 m **não** limita o
mapa a 96 m — quem manda é o `max_range`.

### range_max_factor

```ini
mapper_mapping_mode_on_velodyne_range_max_factor    5.0
```

Divide o `range_max` **apenas dentro de intervalos angulares** definidos por
`mapper_mapping_mode_on_velodyne_range_max_factor_num_intervals` e `..._intervals`
(`change_sensor_rear_range_max()`, `mapper.cpp:246`). Serve para encurtar o alcance atrás do
carro, onde a carroceria atrapalha.

Sem os intervalos configurados, `num_intervals` fica no default negativo, o laço não roda, e
`current_range_max = range_max` em todas as direções — o fator **não tem efeito**. É o caso
do `carmen.ini` da IARA hoje.

### Resolução e tamanho da grade

```ini
mapper_map_grid_res    0.2      # metros por celula
mapper_map_width       210.0
mapper_map_height      210.0
```

`0.2 m` é o padrão. Baixar para `0.1` dobra a memória e o custo por scan, e só vale a pena se
a pose for boa o bastante para justificar — com pose de ±0,3 m, célula de 10 cm só registra
o erro com mais detalhe.

---

## 4. O modelo de ocupação — a matemática

Passados os dois filtros, o ponto entra no modelo que decide **obstáculo ou chão**. Ele não
olha o ponto isolado: compara cada raio com o **raio de baixo**.

### A ideia

Dois raios vizinhos, com elevações diferentes, batendo no chão plano, caem a distâncias
previsíveis um do outro. Se a distância medida entre eles for **menor** que a prevista, é
porque alguma coisa vertical interrompeu o de cima: obstáculo.

```
          sensor
            |\
            | \  raio i-1
            |  \
            |   \  raio i
            |    \
   ─────────┴─────x───x─────────  chao
                  |← →|
                  delta_ray medido
```

### As fórmulas

`carmen_prob_models_compute_expected_delta_ray(h, r1, theta)`:

```c
a     = r1 * sin(acos(h / r1));            // projecao do raio anterior no chao
alpha = asin(a / r1);
expected_delta_ray = (sin(alpha + theta) * h) / sin(90 + alpha + theta) - a;
```

onde:

| símbolo | é |
|---|---|
| `h`     | altura do sensor acima do chão (2,112 m na IARA) |
| `r1`    | comprimento do raio anterior |
| `theta` | diferença de elevação entre os dois raios, em radianos |

Depois:

```c
delta_ray         = ray_size2 - ray_size1;                      // medido no chao
obstacle_evidence = (expected_delta_ray - delta_ray) / expected_delta_ray;
obstacle_evidence = min(obstacle_evidence, 1.0);
```

`obstacle_evidence` é adimensional: `0` = exatamente o esperado para chão plano; `1` = os
dois raios caíram no mesmo ponto, parede vertical.

Se `delta_ray > expected_delta_ray` (os raios se afastaram **mais** que o previsto — buraco,
descida) o retorno é `log_odds_l0`: neutro, nem livre nem ocupado.

### Da evidência para log-odds

```c
p_0        = exp(-1.0 / sigma);
p_obstacle = (exp(-(1 - obstacle_evidence)^2 / sigma) - p_0) / (1 - p_0);
log_odds   = log(p_obstacle / (1 - p_obstacle));      // teto em 37.0
```

`sigma` é o `unexpeted_delta_range_sigma`. É uma gaussiana centrada em
`obstacle_evidence = 1`, normalizada para dar 0 quando a evidência é 0.

### Os parâmetros

```ini
mapper_lidar0_unexpeted_delta_range_sigma   0.45
mapper_lidar0_locc                          5.7     # log-odds somado quando ocupado
mapper_lidar0_lfree                        -5.0     # log-odds somado quando livre
mapper_lidar0_l0                            0.0     # neutro
```

**`unexpeted_delta_range_sigma` — a sensibilidade.**
Menor = mais exigente, só marca obstáculo quando a evidência é quase 1. Maior = aceita
evidência fraca. Se o mapa está **sujo**, com obstáculo onde não há, baixe (0,45 → 0,30).
Se está **furado**, perdendo poste e guia, suba (0,45 → 0,60). Mexa de 0,05 em 0,05 e
refaça o mesmo trecho de log.

**`locc` / `lfree` — a inércia.**
São somados na célula a cada evidência. Valores altos fazem o mapa reagir rápido e ficar
sensível a pose ruim; baixos deixam o mapa lento e limpo.

A razão entre eles é o que importa: com `locc 5.7` e `lfree -5.0` (quase simétricos), uma
observação de livre quase cancela uma de ocupado. Com `lfree -0.6` — o valor antigo — um
obstáculo visto uma vez precisava de ~9 observações de livre para sumir, o que preserva
obstáculo real mas também preserva erro.

- mapa com **rastro de objeto que já passou**: `lfree` mais negativo
- mapa **apagando obstáculo real**: `lfree` menos negativo, ou `locc` maior

**`mapper_lidar0_mode`.**

```ini
mapper_lidar0_mode   0   # 0: main, 1: force_obstacle_height, 2: 0+1, 3: 2+no_raycast
```

`0` é o normal. `3` desliga o raycast (não carimba livre ao longo do raio) — útil para LiDAR
auxiliar que só deve acrescentar obstáculo, nunca limpar.

### ray_index_difference / use_index_difference

```ini
mapper_lidar0_ray_index_difference    0
mapper_lidar0_use_index_difference    off
```

Com `use_index_difference on`, o modelo compara cada raio com outro afastado de
`ray_index_difference` índices, em vez do vizinho imediato (`prob_map.cpp:1538`). Serve para
LiDAR de resolução vertical muito fina, onde raios adjacentes ficam próximos demais e a
diferença medida some no ruído.

Fica `off` por padrão. Ligar exige calibrar o número; o único exemplo calibrado no
repositório é o `mapper_lidar8_ray_index_difference 15`.

---

## 5. Deskew — a nuvem girada

```
 mapper ... -mapping_mode on -interpolate_by_globalpos on
```

Durante os ~50 ms de uma varredura o carro se move. O mapper corrige isso interpolando a
pose ao longo do scan:

```c
dt1 = points_timestamp - robot_timestamp - N * dt;
dt2 = j * dt;                                   // j = indice do tiro
pose_j = interpolacao(pose, dt1 + dt2, ...)
```

Sem `-interpolate_by_globalpos on`, a interpolação usa o **modelo de bicicleta** (`v`, `phi`):
estima a taxa de guinada a partir do esterço. Bom em reta, ruim em curva — e o erro de
guinada aparece no mapa como a nuvem **girada um pouco**, borrando parede e poste.

Com a flag ligada e as poses vindo de um SLAM, a guinada real está na diferença entre duas
`globalpos` seguidas, e é ela que é usada. Só tem efeito em `-mapping_mode on`.

> Flag escrita errado passa em silêncio no mapper — ele ignora e sobe normalmente. Se o mapa
> não mudar, confira a grafia antes de qualquer outra coisa.

---

## 6. Diagnóstico — antes de mexer em valor

### O que o mapper imprime na subida

```
[mapper] lidar0 (OT-128): correcao HORIZONTAL de azimute LIGADA -- 97 de 128 canais com desvio, faixa -3.209..4.681 graus
```

`97 de 128` significa que 31 canais estão sem desvio de azimute (seção 7). Se disser
`DESLIGADA`, o modelo do LiDAR não está na lista do `needs_horizontal_correction()`
(`src/velodyne/velodyne_interface.cpp`).

### Ver a saída dos módulos

O `localize_ackerman` — e vários outros módulos do CARMEN — imprimem **tudo em stderr**. No
process, `> caco_localize.txt` sozinho deixa o arquivo em **0 bytes mesmo com o módulo
funcionando**. Use sempre:

```
 localize   SLAM   1   0   ./localize_ackerman -mapping_mode on > caco_localize.txt 2>&1
```

### A ordem de investigação

Antes de calibrar qualquer coisa, confirme que a **pose** está boa. Mapa borrado por pose
ruim é indistinguível, a olho, de mapa borrado por parâmetro errado — e nenhum ajuste de
`sigma` conserta pose errada.

1. as poses cobrem o log inteiro? (`poses_opt_to_carmen.sh` imprime a sobreposição)
2. o `MAP_X`/`MAP_Y` do process é a primeira pose **deste** log?
3. o mapa aparece no `navigator_gui2` no lugar certo?
4. só então: banda de altura → alcance → sigma → locc/lfree

---

## 7. Limitações conhecidas

Duas coisas afetam a qualidade da grade neste LiDAR e **não se resolvem por parâmetro**.

### Tabela de azimute incompleta

`lidar0_horizontal_angles` tem **97 dos 128** valores. Os 31 canais restantes ficam com
desvio de azimute 0 enquanto os vizinhos estão deslocados em até ±4,7°.

E são justamente os anéis de baixo — elevação de −9,27° a −24,68°, que batem no chão de
**4,6 a 12,9 m** do carro. Ou seja: o grosso do que pinta a grade.

Os 31 valores têm que vir da correção de fábrica da unidade (arquivo da Hesai ou leitura
pela rede). São calibração por canal: não dá para derivar dos grupos de canais nem copiar de
outra unidade — conferido, os desvios variam dentro de cada grupo e diferem entre sensores.

### Vizinho errado no modelo de ocupação

O modelo compara cada raio com `ray_index - 1`, supondo que os dois diferem só em elevação e
caem no mesmo azimute. No OT-128 os canais 40–103 disparam em grupos intercalados de passo
3, então o `j-1` está a **4,5°–7,8° de azimute** — 1,6 a 2,7 m de separação lateral a 20 m.

O `delta_ray` medido passa a refletir a estrutura horizontal da cena, não a geometria
vertical, e o `obstacle_evidence` vira ruído. **66 dos 128 canais** estão pareados errado,
na faixa de elevação −0,26° a −11,5°.

O `.ini` já declara a estrutura de disparo:

```ini
lidar0_number_of_channel_groups   11
lidar0_channel_group0   0 1 2 ... 39
lidar0_channel_group1   40 43 46 49 52 55 58 61
...
```

mas **nenhum módulo do CARMEN lê essas chaves** — são dado morto hoje. Ler os grupos e
escolher o vizinho dentro do mesmo grupo (Δazimute de 0,05°–0,14°) é o que falta.

---

## 8. Receita rápida

| sintoma no mapa | onde mexer |
|---|---|
| copa de árvore, marquise, viaduto viram obstáculo | `mapper_lidar0_unsafe_height_above_ground` ↓ |
| guia e obstáculo baixo somem | `mapper_lidar0_safe_height_from_ground` ↓ |
| chão sujo em terreno irregular | `mapper_lidar0_safe_height_from_ground` ↑ |
| faixas de "livre" atravessando prédio | `lidar0_max_range` ↓ |
| obstáculo onde não há | `mapper_lidar0_unexpeted_delta_range_sigma` ↓ |
| poste e guia não aparecem | `mapper_lidar0_unexpeted_delta_range_sigma` ↑ |
| rastro de carro que já passou | `mapper_lidar0_lfree` mais negativo |
| obstáculo real sendo apagado | `mapper_lidar0_lfree` menos negativo, ou `locc` ↑ |
| nuvem girada, borrão em curva | `-interpolate_by_globalpos on` |

Sempre no mesmo trecho de log, um parâmetro por vez, apagando os `.map` antes de cada
rodada — o mapper **funde** dados novos nos blocos existentes:

```
rm -f ../data/iara/geodata/<LOG_NAME>/map_files/*.map
```

E lembre de espelhar no `localize_ackerman_*` o que mudar na banda de altura.
