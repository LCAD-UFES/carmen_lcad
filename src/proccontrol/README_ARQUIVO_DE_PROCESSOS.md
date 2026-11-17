# O arquivo de processos do `proccontrol`

Este documento descreve a sintaxe do arquivo de processos, a linha de comando do
`proccontrol` e — o ponto que mais gera dúvida — **como os caminhos (paths) que
aparecem no arquivo são resolvidos na hora de iniciar um módulo**.

## Índice

[1. Linha de comando](#1)<br>
[2. Formato do arquivo de processos](#2)<br>
[3. Variáveis: `SET` e `${VAR}`](#3)<br>
[4. Precedência entre linha de comando e arquivo](#4)<br>
[5. Como o path de um módulo é resolvido](#5)<br>
[6. Comentários](#6)<br>
[7. Validação de formato](#7)<br>
[8. Limites](#8)<br>

<a name="1"></a>
## 1. Linha de comando

```
proccontrol [-SET VAR=VALOR]... <arquivo_de_processos>
```

**O arquivo de processos é sempre o último argumento.** As opções vêm antes dele.
A forma tradicional, com o arquivo como único argumento, continua valendo:

```bash
proccontrol process-volta_da_ufes.ini
proccontrol -SET MAP_PATH=/dados/mapa_novo process-volta_da_ufes.ini
proccontrol -SET MAP_PATH=/dados/mapa_novo -SET LOG_PATH=/dados/log.txt process-x.ini
```

Sem nenhum argumento, o `proccontrol` procura `process.ini` no diretório corrente
e, se não achar, `../race/src/process.ini`.

<a name="2"></a>
## 2. Formato do arquivo de processos

Cada linha de módulo tem **exatamente cinco campos**, separados por espaços ou
tabulações:

```
module_name   group_name   requested_state   watch_heartbeats   command_line
```

| campo | significado |
|---|---|
| `module_name` | nome do módulo; é por ele que `proccontrol_setmodule` e o `proccontrol_gui` o referenciam |
| `group_name` | grupo ao qual o módulo pertence; usado por `proccontrol_setgroup` |
| `requested_state` | `1` inicia o módulo junto com o `proccontrol`; `0` deixa parado |
| `watch_heartbeats` | `1` mata e reinicia o módulo se ele deixar de publicar `carmen_heartbeat_message` |
| `command_line` | a linha de comando, do primeiro caractere não-branco até o fim da linha |

O `command_line` é o quinto campo **até o fim da linha** — ele pode conter
espaços, aspas, `;`, pipes e redirecionamentos.

<a name="3"></a>
## 3. Variáveis: `SET` e `${VAR}`

Uma linha que comece com `SET` define uma variável:

```
SET MAP_PATH = ../data/map_volta_da_ufes-20230809
SET LOG_PATH=/dados/log-volta_da_ufes-20230809.txt
SET FRASE = uma frase com espaços      # comentário aqui
SET LOG_PATH=
```

O valor vai do primeiro caractere não-branco depois do `=` até o fim da linha ou
até um `#`, e é aparado nas pontas. Portanto **valores com espaço são válidos**, e
`SET VAR=` define a variável como string vazia.

Uma variável definida é usada como `${VAR}`:

```
 mapper       SLAM      1  0   ./mapper -map_path ${MAP_PATH}
 logger       support   0  0   ./logger ${LOG_PATH}
```

A substituição de `${VAR}` é **textual** e acontece antes de o módulo ser lançado.
Vale para o arquivo inteiro, inclusive dentro de outras linhas `SET`, o que permite
compor caminhos:

```
SET MAP_PATH   = ../data/map_volta_da_ufes-20230809
SET GRAPH_PATH = ${MAP_PATH}/graph.gr
```

Só são substituídas as variáveis definidas em linhas `SET` (ou por `-SET`). Uma
linha `SET` só enxerga as variáveis definidas **acima** dela.

### As variáveis também vão para o ambiente

Além da substituição textual, cada variável definida é exportada para o ambiente
do `proccontrol`. Como o módulo é lançado por um `fork()` seguido de
`execv("/bin/bash", ...)`, ele **herda esse ambiente**. Duas consequências úteis:

1. o próprio módulo pode ler a variável com `getenv("MAP_PATH")`;
2. scripts invocados na `command_line` (um `source venv/bin/activate`, um
   `export PYTHONPATH=...`) enxergam a variável.

### Variáveis de ambiente e expansões do shell

Quem executa a `command_line` é o `bash`. Por isso **toda a sintaxe de expansão do
shell funciona**, e não só o `${VAR}` das linhas `SET`:

```
 aruco   monitors   1  0   ./aruco_theta_tracker -board $CARMEN_HOME/aruco/1-6.yml
 mapper  SLAM       1  0   ./mapper -map_path ${MAP_PATH} -tag ${MAP_PATH##*/}
 deep    detection  1  0   export PYTHONPATH=$CARMEN_HOME/src/x:$PYTHONPATH; source venv/bin/activate; ./modulo
```

`$CARMEN_HOME` e `${MAP_PATH##*/}` acima não são tocados pelo `proccontrol`:
chegam intactos ao `bash`, que os expande. `$CARMEN_HOME` vem do ambiente do
usuário (o `export` do `.bashrc`); `${MAP_PATH##*/}` funciona porque a variável do
`SET` também foi exportada, conforme a seção anterior.

<a name="4"></a>
## 4. Precedência entre linha de comando e arquivo

**A linha de comando vence o arquivo.** As variáveis de `-SET` são registradas
antes de o arquivo ser lido; quando a leitura encontra uma linha `SET` para uma
variável já definida, ela é ignorada e o `proccontrol` avisa:

```
$ proccontrol -SET MAP_PATH=/dados/mapa_novo process-x.ini
Info: setting variable from command line: MAP_PATH = /dados/mapa_novo
Info: variable MAP_PATH of the process file overridden by the command line.
```

Isso permite reaproveitar um arquivo de processos trocando só um caminho, sem
editá-lo. Se o mesmo `-SET` aparecer duas vezes na linha de comando, o último
vence (com um aviso). Dentro do arquivo, uma redefinição da mesma variável é
ignorada — vale a primeira.

<a name="5"></a>
## 5. Como o path de um módulo é resolvido

Esta é a parte que costuma pegar. O `proccontrol` **não faz `chdir()`**: o módulo
é lançado com o mesmo diretório corrente do `proccontrol`. Na prática, isso
significa que **todo caminho relativo do arquivo de processos é relativo ao
diretório de onde você rodou o `proccontrol`** — que, por convenção, é
`$CARMEN_HOME/bin`.

| como aparece no arquivo | como é resolvido |
|---|---|
| `./mapper` | relativo ao diretório corrente → `$CARMEN_HOME/bin/mapper` |
| `../src/carmen-ford-escape.ini` | relativo ao diretório corrente → `$CARMEN_HOME/src/...` |
| `../sharedlib/OpenJAUS/ojNodeManager/bin/ojNodeManager` | idem |
| `str2str` (sem `./` e sem `/`) | procurado no `$PATH` (que inclui `$CARMEN_HOME/bin`) |
| `$CARMEN_HOME/aruco/1-6.yml` | expandido pelo `bash` a partir do ambiente |
| `/dados/log-x.txt` | absoluto, sem ambiguidade |

Ou seja: rodar `proccontrol` de outro diretório que não `bin/` quebra todos os
caminhos relativos do arquivo. A forma correta é sempre:

```bash
cd $CARMEN_HOME/bin
./proccontrol process-x.ini
```

O caminho do **próprio arquivo de processos**, ao contrário, pode ser qualquer um
— relativo ao diretório corrente ou absoluto:

```bash
cd $CARMEN_HOME/bin
./proccontrol ../data/iara/process/process-volta_da_ufes_playback_sensorbox.ini
```

O módulo herda do `proccontrol`, além do diretório corrente, o `PATH`, o
`LD_LIBRARY_PATH` e as variáveis definidas por `SET`/`-SET`. A saída padrão e a de
erro do módulo são redirecionadas para um pipe e publicadas por IPC — é por isso
que elas aparecem no `proccontrol_gui` e no `proccontrol_viewoutput`, e não no
terminal do `proccontrol`.

<a name="6"></a>
## 6. Comentários

Uma linha cujo primeiro caractere não-branco seja `#` é ignorada, e linhas em
branco também. É assim que se desliga um módulo sem apagá-lo.

Dentro da `command_line`, porém, **o `#` não é removido pelo `proccontrol`**: ele
faz parte da linha de comando e quem o interpreta é o `bash`. Isso é o que permite
usar as expansões de parâmetro do shell que dependem de `#` — `${VAR##*/}`,
`${#VAR}` — sem que a linha seja truncada.

A consequência prática é que o `#` de um comentário no fim da linha **precisa
estar precedido de espaço**, que é a regra do próprio `bash`:

```
 playback  support  1  0   ./playback ${LOG_PATH} # -autostart on     <-- certo
 playback  support  1  0   ./playback ${LOG_PATH}# -autostart on      <-- ERRADO
```

Na segunda forma o `bash` não vê um comentário: vê um único argumento terminado em
`#`, e o `playback` recebe um nome de arquivo que não existe.

<a name="7"></a>
## 7. Validação de formato

O `proccontrol` valida cada linha de módulo antes de aceitá-la e **aborta**, com o
número da linha, se ela não tiver os cinco campos:

```
LINE #23: INVALID FORMAT:    mapper SLAM 1
```

Um erro comum que isso pega: uma variável que expande para vazio e era o único
conteúdo do `command_line` — a linha fica com quatro campos e o arquivo é
rejeitado, em vez de o módulo ser lançado sem argumento nenhum.

Linhas `SET` malformadas geram aviso e são ignoradas, sem abortar.

<a name="8"></a>
## 8. Limites

| limite | valor |
|---|---|
| módulos por arquivo | 100 (`MAX_PROCESSES`) |
| variáveis (`SET` + `-SET`) | 256 (`MAX_VARS`) |
| tamanho do nome de uma variável | 255 caracteres |
| tamanho do valor de uma variável | 255 caracteres |
| tamanho de uma linha lida do arquivo | 1000 caracteres |
| tamanho da `command_line` | 1000 caracteres |
