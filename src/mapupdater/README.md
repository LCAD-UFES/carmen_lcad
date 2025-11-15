# MapUpdater - Life Long SLAM

O **MapUpdater** é um módulo do sistema de autonomia CARMEN-LCAD que implementa **Life Long SLAM** (Simultaneous Localization and Mapping contínuo). Este módulo mantém e atualiza mapas de forma contínua durante a operação de veículos autônomos, permitindo que o sistema se adapte a mudanças no ambiente sem intervenção humana.

## Visão Geral

O MapUpdater trabalha em conjunto com os módulos `localizer_ackerman` e `map_server` para realizar:

- **Localização**: Estimação contínua da pose do veículo
- **Mapeamento**: Atualização contínua do mapa do ambiente
- **Fusão de Mapas**: Integração de observações atuais com mapas offline existentes

O módulo compara o mapa atual (gerado em tempo real) com o mapa offline (armazenado) e atualiza o mapa offline quando detecta mudanças significativas no ambiente.

## Como Compilar

### Compilação Normal

```bash
cd /home/claudine/carmen_lcad/src/mapupdater
make
```

### Compilação em Modo Debug

```bash
cd /home/claudine/carmen_lcad/src/mapupdater
make debug
```

O executável será gerado em `$(CARMEN_HOME)/bin/mapupdater`.

## Como Usar

### Execução Básica

O MapUpdater deve ser executado a partir do diretório `bin`:

```bash
cd $(CARMEN_HOME)/bin
./mapupdater -map_path ../data/mapper_teste2 -map_x 7757721.8 -map_y -363569.5 -block_map on
```

### Parâmetros de Linha de Comando

| Parâmetro | Tipo | Descrição | Exemplo |
|-----------|------|-----------|---------|
| `-map_path` | string | Caminho para o diretório onde os mapas são armazenados | `../data/mapper_teste2` |
| `-map_x` | double | Coordenada X inicial do mapa (UTM) | `7757721.8` |
| `-map_y` | double | Coordenada Y inicial do mapa (UTM) | `-363569.5` |
| `-block_map` | on/off | Habilita o uso de mapas em blocos (block maps) | `on` |
| `-map` | string | Caminho para um arquivo de mapa específico (opcional) | `../data/map.ini` |
| `-publish_grid_mapping_map_at_startup` | on/off | Publica o mapa no início (opcional) | `off` |

### Exemplo Completo

```bash
cd $(CARMEN_HOME)/bin
./mapupdater \
    -map_path ../data/mapper_teste2 \
    -map_x 7757721.8 \
    -map_y -363569.5 \
    -block_map on
```

## Parâmetros de Configuração (carmen.ini)

O MapUpdater utiliza vários parâmetros configuráveis no arquivo `carmen.ini`:

### Parâmetros do MapUpdater

| Parâmetro | Tipo | Descrição | Padrão |
|-----------|------|-----------|--------|
| `mapupdater_percentage_change_for_update` | double | Percentual de células que devem mudar para forçar atualização | - |
| `mapupdater_max_log_odds` | double | Valor máximo de log-odds para considerar célula ocupada | - |
| `mapupdater_min_log_odds` | double | Valor mínimo de log-odds para considerar célula livre | - |
| `mapupdater_max_count` | double | Número mínimo de observações para considerar mudança válida | - |
| `mapupdater_strenght_dacay` | double | Limite de reversões (occupied ↔ free) para forçar atualização | - |

### Parâmetros do Map Server

| Parâmetro | Tipo | Descrição |
|-----------|------|-----------|
| `map_server_map_grid_res` | double | Resolução do grid do mapa (metros) |
| `map_server_map_width` | double | Largura do mapa (metros) |
| `map_server_map_height` | double | Altura do mapa (metros) |
| `map_server_initial_waiting_time` | double | Tempo de espera inicial (segundos) |

### Parâmetros do Robot

| Parâmetro | Tipo | Descrição |
|-----------|------|-----------|
| `robot_collision_file` | string | Caminho para arquivo de modelo de colisão do robô (opcional) |

**Nota**: Se o arquivo de colisão não for necessário, você pode comentar a linha no `carmen.ini` ou tornar o parâmetro opcional no código.

## Como Funciona

### Arquitetura

1. **Inicialização**: O MapUpdater carrega o mapa offline inicial e configura os sensores (principalmente Velodyne).

2. **Processamento Contínuo**:
   - Recebe mensagens de `globalpos` do localizador
   - Recebe dados do Velodyne
   - Atualiza o mapa atual com novas observações
   - Compara o mapa atual com o mapa offline

3. **Atualização do Mapa**:
   - Detecta mudanças significativas (reversões de ocupação ou confirmações)
   - Atualiza o mapa offline quando o número de mudanças excede o threshold
   - Publica o mapa atualizado para outros módulos

4. **Gerenciamento de Blocos**:
   - Quando o robô se move para uma nova região, carrega o bloco de mapa correspondente
   - Salva automaticamente os blocos modificados

### Algoritmo de Atualização

O MapUpdater detecta dois tipos de mudanças:

1. **Reversões**: Quando uma célula muda de ocupada para livre (ou vice-versa)
   - Requer: `current_count > max_count` e log-odds acima/abaixo dos thresholds

2. **Confirmações**: Quando uma célula mantém o mesmo estado mas com mais evidências
   - Requer: `current_count > max_count` e mais observações que o mapa offline

A atualização é forçada quando o número de reversões excede `mapupdater_strenght_dacay`.

## Debug

### Usando GDB

```bash
cd $(CARMEN_HOME)/bin
gdb ./mapupdater
(gdb) set args -map_path ../data/mapper_teste2 -map_x 7757721.8 -map_y -363569.5 -block_map on
(gdb) run
```

### Usando VS Code / Cursor

O projeto inclui configurações de debug no diretório `.vscode/`:

1. Abra o diretório `src/mapupdater` no VS Code/Cursor
2. Instale a extensão **C/C++** (Microsoft)
3. Defina breakpoints no código
4. Pressione `F5` ou vá em Run > Start Debugging
5. Selecione "Debug MapUpdater"

A configuração compila automaticamente com `make debug` antes de iniciar o debugger.

## Troubleshooting

### Erro: "Can not load Col File"

Este erro ocorre quando o arquivo de modelo de colisão do robô não é encontrado.

**Soluções:**

1. **Configurar o arquivo de colisão** no `carmen.ini`:
   ```ini
   robot_collision_file argos/argos_col.txt
   ```

2. **Criar um arquivo de colisão simples** (se não tiver um):
   ```
   1
   0
   0.0 0.0 0.5 0
   ```
   Salve em `$(CARMEN_HOME)/bin/meu_robo/meu_robo_col.txt` e configure no `.ini`.

3. **Tornar o parâmetro opcional** (modificando o código em `prob_map.cpp`):
   - Mude `carmen_param_allow_unfound_variables(0)` para `(1)`
   - Mude o último parâmetro de `1` para `0` na definição do parâmetro
   - Adicione verificação `if (poly_file == NULL) return 0;`

### Erro: "mapupdater: could not get an offline map at startup!"

- Verifique se o caminho `-map_path` está correto
- Verifique se existem mapas no diretório especificado
- Verifique se as coordenadas `-map_x` e `-map_y` estão corretas
- Certifique-se de que `-block_map on` está habilitado se usar mapas em blocos

### O mapa não está sendo atualizado

- Verifique os parâmetros `mapupdater_*` no `carmen.ini`
- Ajuste `mapupdater_strenght_dacay` para um valor menor se necessário
- Verifique se o veículo está se movendo (velocidade > 0.05 m/s)

## Dependências

O MapUpdater depende de:

- `localizer_ackerman`: Para fornecer a pose global do veículo
- `map_server`: Para gerenciar e publicar mapas
- `mapper`: Para processar dados do Velodyne
- `velodyne_interface`: Para receber dados do sensor LiDAR

## Estrutura de Arquivos

```
mapupdater/
├── mapupdater_main.cpp    # Código principal
├── fastslam.h             # Definições do FastSLAM
├── Makefile                 # Sistema de build
├── README.md               # Este arquivo
└── .vscode/                # Configurações de debug
    ├── launch.json
    └── tasks.json
```

## Referências

- CARMEN-LCAD: https://github.com/LCAD-UFES/carmen_lcad
- Sistema de autonomia desenvolvido no Laboratório de Computação de Alto Desempenho (LCAD) da UFES

## Autores

Desenvolvido como parte do sistema de autonomia CARMEN-LCAD para veículos autônomos.

