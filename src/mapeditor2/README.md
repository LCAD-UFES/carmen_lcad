# Map Editor 2

Esse módulo é um editor de mapas estilo Photoshop. Ele permite visualizar e editar mapas de ocupação (grid maps) usados em robótica, oferecendo ferramentas de desenho, menus de edição e suporte a formatos gráficos.

## Dependências e Instalação

Como este projeto utiliza uma arquitetura híbrida (Backend em C e Frontend em Python), a instalação é dividida em duas etapas.

### 1. Dependências do Sistema (C/C++)
O núcleo de leitura e salvamento é compilado em C utilizando um `Makefile`. Analisando as flags de compilação (`IFLAGS` e `LFLAGS`), o programa depende das seguintes bibliotecas:

* **Bibliotecas do Sistema:** GTK+ (interface gráfica legada) e OpenCV (processamento de imagem).

* **Bibliotecas CARMEN:** libmap_interface, libmap_io, libglobal_graphics, libglobal, libipc.

Para instalar as dependências em um sistema Linux (como Ubuntu/Debian), execute:

```
sudo apt-get update
sudo apt-get install libgtk2.0-dev libopencv-dev
```
### 2. Dependências do Python
A nova interface gráfica requer as seguintes bibliotecas Python:

```
pip install numpy PyQt5
```

## Compilação e Execução

### 1. Compilar os Binários em C
Antes de rodar o editor, é necessário compilar os utilitários C (`carmen_read`, `carmen_save`) que o Python utiliza para ler e escrever os mapas corretamente.

Navegue até a pasta do projeto (ex: `src/mapeditor2`) e rode:

```
make
```

* **Nota:** Certifique-se de que o comando make gerou os executáveis sem erros.

### 2. Rodar o Editor
Com os binários compilados no mesmo diretório, execute o script principal em Python:

```
python3 mapeditor2.py
```

### 3. Navegar pelo editor
O Map Editor 2 ela vem com as mesma feramentas do mapeditor, com a posibilidade de abrir um diretorio com todos os `.map`, os comando basicos.

* Na barra de ferramentas no topo do editor ele contem o botão de abrir o mapa **`Abrir Pasta`**, naveguer ate o diretorio `~/carmen/data/Diretorio_mapa_completo`, ele vai carregar o mapa por completo.

* O mapa vem com 3 cores, o azul demostrando o espaço desconhecido `(-1)`, o branco a area livre `(0)` e a preta a ocupada `(1)`.

* A três tipo tipo de pincel o que desenha obstáculo, o da borracha e o Desconhecido.

* O editor tem uma função que assim que começa a edita ele criar camada em cima do mapa `EX: fez um risco`, ser vc clica no botão `Desfazer` ele apaga a camada mais recente que vc criou.

* Quando vc finalizar o trabalho e vai salva, ele pedir para vc seleciona a pasta que gostaria de salva, e no caso para economizar memoria e processos ele so modificar e salva as `.map` que foi modificado, as demais so copia para nova pasta.

* O editor vem com um sistema que assim que vc editar uma .map ele da uma condição de evita que vc feche o trabalho sem salva, uma dupla verificação.

## Modos de edição

O combo **Modo de Ação**, na barra de cima, troca o que o clique no mapa faz. As barras de
ferramentas e os painéis laterais aparecem conforme o modo.

| Modo | O que faz |
|---|---|
| **Pintar Mapa** | pincéis de obstáculo / borracha / desconhecido no grid |
| **Editar Caminho (RDDF)** | arrasta as bolinhas vermelhas da trajetória; suaviza; salva; **desenha um RDDF do zero** |
| **Editar Locais (Anotações)** | cria e move os `places` (anotação tipo 13) usados por `set course to place <nome>` |
| **Criar/Editar Missão** | escreve, valida e salva os arquivos de missão do `task_manager` |

## Camadas: o que aparece no mapa

Na barra de cima, ao lado do modo, há as caixas **Mostrar: RDDF | Grafo | Locais | Missão**.

Elas controlam a visibilidade **independente do modo de edição**. O modo passa a decidir só o
que pode ser *arrastado* — antes, trocar de modo escondia o RDDF e os locais sozinho, e eles
sumiam do mapa sem motivo aparente.

## Grafo (.gr)

O grafo é a malha que o `route_planner` realmente percorre. Na segunda linha da barra do modo
RDDF há o combo **Grafo**, **Abrir .gr** e **Remover**; ao usar **Abrir Pasta**, o editor
procura sozinho uma pasta `graphs/` no ambiente.

Ele é desenhado em verde: arestas como linhas, nós como pontinhos. Um grafo de 922 nós e 930
arestas carrega e desenha em ~20 ms, porque tudo vai em dois `QGraphicsPathItem` em vez de
milhares de itens soltos.

Com o grafo carregado, **a rota do preview de missão passa a segui-lo** em vez do RDDF — é o
traçado mais fiel ao que o robô vai fazer.

## Criar um RDDF do zero

No modo **Editar Caminho (RDDF)**:

1. **Desenhar Novo** — o cursor vira uma cruz. Clique no mapa marcando os vértices do caminho.
   **Apagar Último Ponto** desfaz o último clique; o botão vira *Cancelar Desenho* enquanto isso.
2. Ajuste **Espaç.(m)** (distância entre os pontos gerados ao longo do traço, padrão 0,5 m) e
   **Vel.(m/s)** (velocidade gravada em cada ponto, padrão 2,777778 = 10 km/h).
3. **Finalizar Desenho** — o editor interpola o polígono clicado, calcula o `theta` de cada
   ponto olhando para o seguinte, e o caminho vira o RDDF atual. Daí já dá para arrastar as
   bolinhas e usar **Suavizar**.
4. **Salvar RDDF** e, se quiser navegar por ele, **Gerar Grafo (.gr)** (precisa da `central`
   do carmen rodando).

O arquivo sai no formato de 6 colunas do carmen: `x  y  theta  v  phi  timestamp`.

## Criar e editar missões

No modo **Criar/Editar Missão** abre um painel à direita.

- **Pasta...** aponta para a pasta `missoes/` do veículo — a mesma que o `navigator_gui2` lista
  no combo *Mission* (parâmetro `navigator_panel_missions_folder` do `.ini`). Ao usar
  **Abrir Pasta**, o editor procura sozinho uma pasta `missoes/` no ambiente carregado.
- O combo de cima lista os `.txt` da pasta; escolher um abre no editor de texto.
- **Nova** começa uma missão a partir do esqueleto; **Modelo** insere o esqueleto onde o cursor
  estiver; **Excluir** apaga o arquivo aberto.
- **Destino pelo mapa** — clique no mapa onde o robô deve chegar e **arraste** para definir a
  orientação; o editor insere `set course to <x> <y> <theta>` seguido de `go` na posição do
  cursor de texto.
- **Inserir place** insere `set course to place <nome>` + `go`, usando os locais carregados no
  modo *Editar Locais*.
- **Verificar** procura as armadilhas que fazem uma missão falhar em silêncio: linha começando
  com espaço (que o interpretador ignora), `go after set course to` (task quebrada, não publica
  o destino), vírgula dentro do nome da task, `stop` sem navegação (trava para sempre), label
  inexistente (mata a missão com `exit(1)`) e falta da quebra de linha final.
- **Salvar** / **Salvar como...** gravam garantindo a quebra de linha no fim do arquivo.

### Ver a missão no mapa

Ao abrir uma missão — e a cada edição, com 250 ms de folga — o editor **desenha no mapa o que
ela vai fazer**: os destinos na ordem de execução, numerados, com uma seta mostrando a
orientação (`theta`) de cada um, e a rota ligando um ao outro. **Ver no mapa** redesenha na mão.

| No mapa | O que é |
|---|---|
| bolinha branca marcada **S** | `set robot pose` — a pose inicial |
| bolinha azul **1, 2, 3…** | `set course to` (por coordenada ou `place`) |
| bolinha roxa | `park at` / `park truck and semi-trailer at` |
| bolinha amarela | `set final goal` |
| bolinha vermelha | `go after set course to` — a task quebrada, com aviso ao lado |
| linha azul grossa | a rota real, seguindo o grafo (.gr) se houver, senão o RDDF |
| linha cinza tracejada | reta — não há RDDF carregado, ou o destino está longe dele |

Carregue o grafo (ou, na falta dele, o RDDF) antes, para ver o traçado de verdade em vez da reta.
Um `place` citado na missão que não exista nas anotações é avisado na barra de status, com o
número da linha.

A leitura é de cima para baixo: desvios por label não são simulados.

Para a missão nova aparecer no combo *Mission* do `navigator_gui2`, salve em `missoes/` e
reinicie o `navigator_gui2` — ele monta o combo na partida.

⚠️ O `navigator_gui2` lista **todo** arquivo da pasta que não comece com `.` e corta os 4
últimos caracteres do nome. Em `missoes/` só pode haver `.txt` de missão; sub-missões (as
chamadas com `call`) vão em uma pasta ao lado.
