# carmen_lcad

Version of the Carmen Robot Navigation Toolkit developed by LCAD for IARA - the Intelligent Autonomous Robotic Autonomobile.

This software is being developed with the support of DI/LCAD/UFES and is financed in part by Fundação de Amparo à Pesquisa do Espírito Santo – Brazil (FAPES), Conselho Nacional de Desenvolvimento Científico e Tecnológico – Brasil (CNPq), and Coordenação de Aperfeiçoamento de Pessoal de Nível Superior – Brasil (CAPES).

## Instalação

### ➡️ Ubuntu 26.04 LTS — [**tutorial completo, aqui no repositório**](doc/migracao_ubuntu26/install/INSTALL_UBUNTU_26.04.md)

Esta é a versão recomendada e o tutorial mais atual. Ele fica **dentro do próprio
repositório** (não na wiki).

Resumo do caminho (o passo a passo com todos os detalhes está no tutorial):

```bash
git clone https://github.com/LCAD-UFES/carmen_lcad.git ~/carmen_lcad
# 1. adicionar o bloco #CARMEN no ~/.bashrc  (CARMEN_HOME, PATH, LD_LIBRARY_PATH)
# 2. instalar os pacotes apt  (build tools, Boost, OpenCV/PCL/g2o, GTK & cia.)
# 3. instalar o GtkGLExt da fonte, se ainda não estiver na máquina
cd $CARMEN_HOME/src
./configure --nojava --nocuda --noqt3       # Enter em todas as perguntas
make -j$(nproc)
```

Junto do tutorial estão os outros documentos da migração para o 26.04 — úteis quando
algo não compila:

| Documento | Para quê |
|---|---|
| [`install/INSTALL_UBUNTU_26.04.md`](doc/migracao_ubuntu26/install/INSTALL_UBUNTU_26.04.md) | **O tutorial de instalação.** Comece por aqui |
| [`install/00_` a `05_`](doc/migracao_ubuntu26/install/) | O detalhe e o porquê de cada passo do tutorial |
| [`CHANGELOG.md`](doc/migracao_ubuntu26/CHANGELOG.md) | Cada alteração de código/Makefile feita na migração, e por quê |
| [`AUDIT_DEPENDENCIAS.md`](doc/migracao_ubuntu26/AUDIT_DEPENDENCIAS.md) | Nome e versão de cada dependência no 26.04: o que mudou de nome, o que sumiu do apt |
| [`install/05_pendencias_conhecidas.md`](doc/migracao_ubuntu26/install/05_pendencias_conhecidas.md) | O que ficou pendente e por quê |

### Versões anteriores (wiki)

Português:

- [Install for Ubuntu 12.04](http://www.lcad.inf.ufes.br/wiki/index.php/Instala%C3%A7%C3%A3o_Carmen_para_Ubuntu_12.04.3)
- [Install for Ubuntu 16.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-LCAD-on-Ubuntu-16.04-(Portuguese))
- [Install for Ubuntu 18.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-LCAD-on-Ubuntu-18.04-(Portuguese))
- [Install for Ubuntu 20.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-on-Ubuntu-20.04-(Portuguese))

English:

- [Install for Ubuntu 12.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-LCAD-on-Ubuntu-12.04.3-(English))
- [Install for Ubuntu 14.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-LCAD-on-Ubuntu-14.04-(English))
- [Install for Ubuntu 16.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-LCAD-on-Ubuntu-16.04-(English))
- [Install for Ubuntu 18.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-LCAD-on-Ubuntu-18.04-(English))
- [Install for Ubuntu 20.04](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-on-Ubuntu-20.04-(English))

## Rodando

Os `.ini` de parâmetros ficam em `src/`, e os binários rodam a partir de `bin/` — os
caminhos dentro dos `.ini` são relativos a `bin/`.

```bash
cd $CARMEN_HOME/bin
./central
```


```bash
cd $CARMEN_HOME/bin
./proccontrol process-volta_da_ufes_playback_viewer_3D.ini
```
