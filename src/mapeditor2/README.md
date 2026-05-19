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
python3 main.py
```

### 3. Navegar pelo editor
O Map Editor 2 ela vem com as mesma feramentas do mapeditor, com a posibilidade de abrir um diretorio com todos os `.map`, os comando basicos.

* Na barra de ferramentas no topo do editor ele contem o botão de abrir o mapa **`Abrir Pasta`**, naveguer ate o diretorio `~/carmen/data/Diretorio_mapa_completo`, ele vai carregar o mapa por completo.

* O mapa vem com 3 cores, o azul demostrando o espaço desconhecido `(-1)`, o branco a area livre `(0)` e a preta a ocupada `(1)`.

* A três tipo tipo de pincel o que desenha obstáculo, o da borracha e o Desconhecido.

* O editor tem uma função que assim que começa a edita ele criar camada em cima do mapa `EX: fez um risco`, ser vc clica no botão `Desfazer` ele apaga a camada mais recente que vc criou.

* Quando vc finalizar o trabalho e vai salva, ele pedir para vc seleciona a pasta que gostaria de salva, e no caso para economizar memoria e processos ele so modificar e salva as `.map` que foi modificado, as demais so copia para nova pasta.

* O editor vem com um sistema que assim que vc editar uma .map ele da uma condição de evita que vc feche o trabalho sem salva, uma dupla verificação.
