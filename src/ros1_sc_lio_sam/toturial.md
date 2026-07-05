# Tutorial: LT-mapper integrado ao CARMEN

Este tutorial explica como o **LT-mapper** funciona dentro do pipeline do CARMEN, o papel de cada submódulo (SC-LIO-SAM, LT-SLAM, LT-remover e LT-map), a modificação feita para alinhar mapas com pontos de origem opostos, os parâmetros principais de cada arquivo de configuração, e o passo a passo prático de execução — incluindo verificação no CloudCompare e recomendações de hardware.

---

## 1. Visão geral

O **LT-mapper** é um framework modular para mapeamento 3D de longo prazo (lifelong mapping) com LiDAR. Ele resolve três problemas em sequência:

1. **Alinhar múltiplas sessões de mapeamento** (feitas em dias/momentos diferentes) num mesmo referencial.
2. **Detectar o que mudou** entre essas sessões (objetos que apareceram, desapareceram ou são apenas ruído de objetos em movimento).
3. **Gerenciar essas mudanças** para manter um mapa atualizado e/ou um mapa "permanente" limpo.

No seu setup, o pipeline completo dentro do CARMEN é:

```
CARMEN + SC-LIO-SAM  →  LT-SLAM  →  LT-remover + LT-map
   (gera cada sessão)   (alinha)     (limpa e atualiza o mapa)
```

---

## 2. O que cada componente faz

### 2.1 SC-LIO-SAM
É o responsável por gerar a **odometria e o mapa bruto de cada sessão**. É o LIO-SAM (odometria LiDAR-inercial via smoothing/mapping) com o add-on Scan Context, que salva, para cada keyframe, a nuvem de pontos, a pose e um descritor global. Esse conjunto (pose-graph + keyframes) é o que o LT-SLAM vai consumir depois. No seu processo do CARMEN, ele roda via `roslaunch lio_sam run.launch`, salvando os `.pcd` na pasta definida em `SC_LIO_SAM_MAP_PATH`.

### 2.2 LT-SLAM
É o motor de **SLAM multi-sessão**. Ele pega duas sessões (uma "central", já existente, e uma "query", nova) e as alinha num referencial compartilhado, mesmo que cada uma tenha sido gravada com sua própria origem/coordenadas. Ele faz isso por:
- Detecção de loop closure entre sessões via **Scan Context**;
- Refinamento via ICP e busca por proximidade (radius search);
- Otimização via **anchor nodes** (um nó de "offset" entre as sessões), resolvida com GTSAM/iSAM2.

A saída são as poses de cada sessão já no mesmo referencial (os arquivos `*_aft_intersession_loops.txt`).

⚠️ **Limitação importante:** o LT-SLAM só alinha bem trajetórias que cobrem o **mesmo caminho, no mesmo sentido geométrico** (A até B pela mesma rota). Se você tentar alinhar trajetórias genuinamente diferentes, o resultado distorce as poses e o mapa final fica inutilizável.

### 2.3 LT-remover (LT-removert)
Faz a **detecção de mudanças entre as duas sessões já alinhadas**, em duas etapas:
1. **Remoção de pontos altamente dinâmicos (HD)** — carros, pessoas, etc. — usando o algoritmo Removert (remove-então-reverte, baseado em imagens de range com múltiplas janelas).
2. **Detecção de mudanças de baixa dinâmica (LD)** — comparando as duas nuvens via kd-tree para achar pontos que só existem numa das sessões (diferença positiva/PD = apareceu; diferença negativa/ND = desapareceu), com tratamento de oclusão para não apagar pontos estáticos que só estavam "escondidos" numa sessão.

Ele **depende diretamente da saída do LT-SLAM** — é o LT-SLAM que gera as coordenadas/poses alinhadas que o remover precisa para funcionar.

### 2.4 LT-map
É quem **aplica** as mudanças detectadas pelo LT-remover ao mapa central, atualizando-o. No seu fluxo, ele roda junto com o remover (`roslaunch removert run_ltmapper.launch` cobre os dois).

---

## 3. A modificação: alinhar mapas com origem oposta (A→B e B→A)

### 3.1 O problema no LT-SLAM original
No LT-SLAM original, o **anchor node** da sessão "query" (o nó que representa o offset entre o referencial da sessão nova e o da sessão central) começa com um valor inicial "neutro" (perto da identidade) e uma covariância grande, esperando que a detecção de loop via Scan Context, sozinha, convirja para o offset correto durante a otimização.

Isso funciona bem quando as duas sessões percorrem o **mesmo caminho no mesmo sentido**. Mas quando a sessão "query" percorre o **mesmo trajeto físico só que ao contrário** — por exemplo, o mapa 1 foi gravado indo de um ponto A até um ponto B, e o mapa 2 foi gravado voltando de B até A — o ponto de partida da sessão 2 não coincide com o ponto de partida da sessão 1: ele coincide com o **fim** dela, e com um heading (yaw) invertido em torno de 180°. Partindo de um anchor "neutro", o otimizador tende a não convergir (ou converge errado) nesse cenário, principalmente em trajetos longos.

### 3.2 A modificação que você fez no código
Você alterou o código-fonte original do LT-SLAM para que o anchor node da sessão query **aceite um valor inicial explícito** (x, y, z, yaw), em vez do valor neutro padrão. Ou seja, em vez de deixar o otimizador "adivinhar" o offset do zero, você injeta uma estimativa inicial coerente com a geometria real do encontro entre as duas sessões — e é a partir dela que a otimização (via anchor node + loops SC/ICP) refina o alinhamento fino.

Isso é o que permite o cenário A→B / B→A:

```
Mapa 1 (central):  A ────────────────────────► B
Mapa 2 (query):    B ◄──────────────────────── A     (gravado no sentido contrário)
```

Fisicamente, a **origem do mapa 2** (ponto B) coincide com o **fim do mapa 1** (ponto B também) — só que a sessão 2 chega ali "de costas" em relação à sessão 1, com o heading girado ~180°. A receita para calcular o chute inicial é:

1. Pegue a pose do **último nó** da sessão central (mapa 1): `x, y, z, yaw` no ponto B.
2. Use esses mesmos `x, y, z` como posição inicial do anchor da sessão query (mapa 2), já que fisicamente é o mesmo ponto.
3. Some **~180°** ao `yaw`, porque a sessão 2 passa por ali no sentido inverso.

No `params_ltslam.yaml` isso fica:

```yaml
# Preencha com uma estimativa grosseira de onde a sessão "02" começa, no referencial
# da sessão central "01". Se a 02 é o caminho de volta (começa perto de onde a 01
# terminou), pegue a pose do último nó de "01" (x,y,z,yaw) e some ~180 graus no yaw.
query_anchor_init_x: -107.463
query_anchor_init_y: -52.144
query_anchor_init_z: 2.035
query_anchor_init_yaw_deg: 7.007
```

Importante: o anchor continua sendo uma variável otimizável (com covariância grande) — ele **não fica travado** nesse valor. A diferença é que agora ele **começa perto do valor certo**, em vez de começar do zero, o que evita que a otimização convirja para um mínimo local errado quando os pontos de partida/chegada das duas sessões são opostos.

Esse mesmo raciocínio generaliza para qualquer par de sessões: o que importa não é "sempre somar 180°", e sim **identificar o ponto físico onde as duas trajetórias se encontram** (início-com-início, início-com-fim, ou qualquer outro trecho comum) e calcular o `yaw` inicial a partir da diferença de heading real entre as sessões naquele ponto.

### 3.3 Ajustes complementares para esse cenário
Dois parâmetros do Scan Context ajudam quando as sessões são longas e/ou de sentido oposto:
- `scDistThres` (comece testando entre 0.4 e 0.5) — sensibilidade para aceitar um loop entre sessões;
- `scNumCandidates` (comece testando entre 10 e 20) — quantos candidatos de loop são avaliados.

E o `rsSearchRadius` (busca por proximidade entre sessões) só deve ser aumentado **depois** que o anchor já estiver numa posição razoável — aumentá-lo sem corrigir o chute inicial só aumenta falso positivo.

---

## 4. Parâmetros principais de cada arquivo

### 4.1 `params_argos.yaml` (SC-LIO-SAM)
| Grupo | Parâmetros | Para que serve |
|---|---|---|
| Tópicos/Frames | `pointCloudTopic`, `imuTopic`, `lidarFrame`, `baselinkFrame` | Define de onde vêm os dados e os frames do TF |
| Exportação | `savePCD`, `savePCDDirectory`, `localizationMode` | Onde salvar os keyframes/poses da sessão; `localizationMode: true` reusa um mapa existente só para localizar, sem sobrescrever |
| Sensor | `N_SCAN`, `Horizon_SCAN`, `lidarMinRange`/`MaxRange` | Características do LiDAR usado |
| IMU | `imuAccNoise`, `imuGyrNoise`, `imuAccBiasN`, `imuGyrBiasN`, `imuGravity` | Ruído/calibração do IMU (valores de allan variance) |
| Features LOAM | `edgeThreshold`, `surfThreshold`, `edgeFeatureMinValidNum` | Controla o que é aceito como "quina" (edge) vs "superfície plana" — valores mais altos = mais exigente, evita features falsas em ambientes estreitos (corredores) |
| Loop closure | `loopClosureEnableFlag`, `historyKeyframeSearchRadius`, `historyKeyframeFitnessScore` | Detecção de loop **dentro** da própria sessão |

### 4.2 `params_ltslam.yaml`
| Parâmetro | Para que serve |
|---|---|
| `sessions_dir`, `central_sess_name`, `query_sess_name` | Onde estão as sessões e qual é a central/query |
| `save_directory` | Onde salvar as poses alinhadas |
| `loopFitnessScoreThreshold` | Score máximo de ICP para aceitar um loop entre sessões |
| `kNumSCLoopsUpperBound` / `kNumRSLoopsUpperBound` | Limite de loops considerados (Scan Context / busca por raio) |
| `query_anchor_init_x/y/z/yaw_deg` | Chute inicial da posição da sessão query (ver seção 3) |
| `scDistThres`, `scNumCandidates` | Sensibilidade do Scan Context para achar loops entre sessões |
| `rsSearchRadius` | Raio de busca por proximidade entre sessões (só relevante com anchor já razoável) |

### 4.3 `params_ltremovert.yaml` (LT-remover)
| Grupo | Parâmetros | Para que serve |
|---|---|---|
| Saída | `saveMapPCD`, `saveCleanScansPCD`, `save_pcd_directory` | O que salvar e onde |
| Entradas | `central_sess_scan_dir`, `central_sess_pose_path`, `query_sess_scan_dir`, `query_sess_pose_path` | Apontam para os scans e para os `*_aft_intersession_loops.txt` gerados pelo LT-SLAM |
| Sensor | `sequence_vfov`, `sequence_hfov` | Campo de visão vertical/horizontal do LiDAR usado |
| Amostragem | `use_keyframe_gap`, `keyframe_gap` | De quantos em quantos frames processar (aumentar reduz uso de RAM) |
| Lote | `clean_for_all_scan`, `batch_size`, `valid_ratio_to_save` | Processa a sequência inteira em lotes |
| Resolução | `remove_resolution_list`, `revert_resolution_list` | Resolução das imagens de range usadas para detectar/reverter mudanças |
| Sensibilidade estática | `downsample_voxel_size`, `num_nn_points_within`, `dist_nn_points_within` | Quão "rigoroso" é o critério para considerar um ponto estático |
| Performance | `num_omp_cores` | Threads usadas para projeção dos pontos |

---

## 5. Ordem de execução (passo a passo)

### Passo 1 — Gerar o mapa de cada sessão (CARMEN + SC-LIO-SAM)
Rode o proccontrol apontando para o `.ini` de processo, ajustando `LOG_NAME` e `SC_LIO_SAM_MAP_PATH` para cada sessão que for gerar (uma vez para o mapa 1, outra para o mapa 2):

```bash
proccontrol argos/process-argos-sc-lio-sam.ini
```

Esse `.ini` sobe, entre outras coisas, o `playback` do log, o `mapper` do CARMEN e o SC-LIO-SAM (`roslaunch lio_sam run.launch`), salvando os keyframes/poses da sessão na pasta configurada em `SC_LIO_SAM_MAP_PATH`.

### Passo 2 — Alinhar as sessões com o LT-SLAM

```bash
roslaunch ltslam run.launch
```

Configure o `params_ltslam.yaml` com `sessions_dir`, `central_sess_name`, `query_sess_name` e o chute inicial do anchor (seção 3). A saída são os arquivos de pose alinhada, com nome no padrão:

```
nome-do-mapa1_aft_intersession_loops.txt
nome-do-mapa2_aft_intersession_loops.txt
```

### Passo 3 — Verificar o alinhamento no CloudCompare

Antes de seguir para a limpeza, confira visualmente se as trajetórias ficaram alinhadas:

1. Abra os dois arquivos `*_aft_intersession_loops.txt` no CloudCompare como arquivo ASCII;
2. Na configuração de importação, mapeie:
   - **Coluna 3 → X**
   - **Coluna 8 → Y**
   - **Coluna 12 → Z**
3. Compare visualmente as duas trajetórias — elas devem se sobrepor de forma coerente.

Lembre-se da limitação da seção 2.2: isso só funciona bem se as duas sessões percorrem o **mesmo caminho**. Trajetórias diferentes vão aparecer distorcidas mesmo com o LT-SLAM rodando "sem erro".

### Passo 4 — Limpeza do mapa com LT-remover + LT-map

```bash
roslaunch removert run_ltmapper.launch
```

Ajuste o `params_ltremovert.yaml` apontando `central_sess_pose_path` e `query_sess_pose_path` para os arquivos gerados no Passo 2. O LT-remover e o LT-map rodam juntos nesse launch: o remover limpa (HD/LD) e o LT-map aplica a atualização no mapa central.

---

## 6. Recomendações práticas

- **RAM/Swap:** o processo consome muita memória por causa das nuvens de pontos — dependendo do tamanho do mapa, pode consumir toda a RAM disponível. Recomendo **aumentar a swap** antes de rodar. Como referência, no teste com o mapa Honofre ↔ LCAD foi necessário dividir os frames por 2 (`keyframe_gap`) e, mesmo assim, o processo consumiu os 16 GB de RAM + 20 GB de swap disponíveis, por causa do volume de nuvens de pontos.
- **Como saber que terminou:** quando a CPU se estabiliza (uso baixo) mas a memória continua ocupada, geralmente significa que ele já salvou o mapa e só falta encerrar o processo manualmente (Ctrl+C). Depois disso, ele ainda solta outputs adicionais, como os scans e o mapa atualizado.
- **Visualização final:** recomendo usar o CloudCompare para inspecionar o mapa resultante.

---

## 7. Instalando o CloudCompare corretamente

Não instale via `apt` — o pacote do repositório não carrega as bibliotecas necessárias para abrir arquivos `.pcd` (formato PCL) corretamente. Instale via Flatpak:

```bash
flatpak install flathub org.cloudcompare.CloudCompare
```

Mantenha o Flatpak atualizado para garantir que está baixando a versão mais recente do CloudCompare:

```bash
flatpak update
```