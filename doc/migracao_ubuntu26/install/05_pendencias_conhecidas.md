# 5. Pendências conhecidas / próximos passos

O que foi identificado durante a migração e **ainda não foi resolvido** — fica registrado aqui
para não se perder, e para virar o roteiro das próximas sessões.

## a) `proccontrol_gui` ✅ **RESOLVIDO** — reescrito em GTK3

Era o único binário do build principal que ficava de fora (Qt4 + Qt3Support, construído por
`qmake` a partir de um `.pro`). **Foi reescrito em GTK3**, seguindo a mesma solução que o
um fork deste código já usava, e agora é um alvo normal do `Makefile` do módulo — ver
`../CHANGELOG.md`, item **44**.

O que isso muda no ambiente:

- **o CARMEN não depende mais de Qt para nada no build principal** — nada de `qtbase5-dev`,
  `qmake`, `moc` ou `.pro`;
- a dependência nova é `libgtk-3-dev`, que já estava instalada;
- `src/proccontrol/proccontrol_gtk.c` é o fonte; `proccontrol_gui.{h,cpp}` e `proccontrol.pro`
  continuam no repositório mas **não são compilados por nada** (mesma situação num fork deste código).
  Apagá-los é uma limpeza à parte, a decidir.

⚠️ Numa sessão **Wayland**, este GUI (GTK3) usa o backend Wayland, enquanto as outras GUIs do
CARMEN (GTK2) caem no XWayland. Ele não aparece em `xwininfo`/`import` — a janela existe, só
não é um cliente X. Para depurar com ferramentas X11: `GDK_BACKEND=x11 ./proccontrol_gui`.

## b) Módulos que dependem de SDK/lib externa

Ver `../CHANGELOG.md`, item 27. Todos compilam parcialmente (a interface IPC sai; o driver/GUI
não) e voltam sozinhos ao instalar a dependência:

| Módulo | O que falta | Como habilitar |
|---|---|---|
| `bumblebee_basic` | FlyCapture2 (PointGrey/FLIR) | `install/04`, seção 4.4 |
| `traffic_light` | `libwnn` (+ CUDA/Caffe para o detector) | `install/04`, seção 4.4 |
| `tracker` | MAE (`$MAEHOME` + `netcomp`) | `install/04`, seção 4.4 |
| `sensors/kinect` | `libfreenect` | `sudo apt-get install libfreenect-dev` |

Nenhum deles é necessário para subir `central` + `param_daemon` + `proccontrol`.

## c) Módulos fora do `PACKAGES` do `src/Makefile`

> ⚠️ **Não rode `make` neles.** O alvo `export` de cada módulo repõe os symlinks de
> `include/carmen/` para os headers dele. Como esses módulos estão fora do `PACKAGES`
> por terem código desatualizado, os headers são incompatíveis e um `make` ali
> sequestra headers públicos, quebrando módulos que não têm relação nenhuma. Já
> aconteceu com `src/path_planner2`, que repontou `rddf_{messages,interface,util,index}.h`
> de `src/rddf/` para si mesmo. Ver a seção *Quatro armadilhas do build* no
> [`INSTALL_UBUNTU_26.04.md`](INSTALL_UBUNTU_26.04.md), com o comando de conferência.

O `src/` tem ~218 diretórios, mas o `PACKAGES` do `src/Makefile` lista pouco mais de 80. Todo
o resto (`mapper2`, `mapper3`, `mapper_datmo`, `graphslam_*`, `virtual_scan*`, `lpl`,
`laslam`, `polar_slam`, `deep_mapper`, `neural_object_detector*`, `rl_motion_planner`, ...)
**não é construído** e, portanto, **não foi migrado nem testado**. A varredura de OpenCV/Boost
cobriu boa parte deles por tabela (os Makefiles e includes foram corrigidos junto), mas ninguém
compilou — é esperado que apareçam erros quando algum for reativado.

Regra que apareceu duas vezes nesta migração (itens 8 e 28): **um módulo fora do `PACKAGES`
cujo header público é incluído por um módulo de dentro quebra o build numa árvore limpa**,
porque o header nunca é exportado para `include/carmen/`. Se aparecer
`carmen/xxx_interface.h: No such file or directory`, é isso.

## d) Código de terceiros vendorizado — não migrado de propósito

`sharedlib/darknet*`, `sharedlib/ENet/caffe-enet`, `src/semantic_segmentation/caffe-segnet`,
`sharedlib/libsqueeze_seg_v2`, `sharedlib/Deeplab`, `sharedlib/nn_lib`, `src/sensors/minoru`:
árvores de terceiros congeladas, com API de OpenCV/CUDA/Caffe da época. Ficaram **de fora das
varreduras de OpenCV** (só o `pkg-config opencv4` foi aplicado onde já havia), porque migrá-las
é reescrever código que não é do LCAD e que não faz parte do `make` principal.

## e) Bridge ROS (`src/sc_lio_sam`) — **RESOLVIDA**

Era `src/ros1_sc_lio_sam`, com um snapshot antigo do módulo e um `CMakeLists.txt` que ainda
apontava para `/usr/local/carmen_boost`. Foi **refeita a partir da versão de um fork deste
código** (já ajustada para o 26.04) e renomeada para **`src/sc_lio_sam`**. O
`/usr/local/carmen_boost` sumiu junto — essa versão nunca teve a flag.

ROS 1 Noetic não tem pacote para o Ubuntu 26.04; nesta máquina ele foi compilado da fonte,
com a receita em `~/ros_src/NOTAS.md`.

Build próprio via `catkin_make`, fora do `make` do `src/` (ele linka contra `$CARMEN_HOME/lib`,
então rode o `make` do `src/` antes):

```bash
source /opt/ros/noetic/setup.bash
cd $CARMEN_HOME/src/sc_lio_sam
catkin_make -j$(nproc) -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

Compila limpo (exit 0, ~1m40s, 12 executáveis em `devel/lib/`). O que precisou ser desligado
ou contornado por não existir no CARMEN — IMU embutido do LiDAR, `horizontal_angles_deltas` do
`carmen_lidar_config`, `theta`/`valid` do `carmen_gps_xyz_message`,
`..._initialize_gaussian_command_host()` — está detalhado na seção *Notas de portabilidade*
do `src/sc_lio_sam/README.md`. **Nada disso mexeu em código do CARMEN**: tudo ficou
dentro do módulo, atrás de uma chave, para poder ser religado.

O `src/ros1-lt-mapper` citado antes nunca existiu neste repositório (o `lt-mapper` vem dentro
do próprio workspace, em `src/sc_lio_sam/src/lt-mapper/`, e compila junto).

## f) `ubuntu_packages/` na raiz do repo

Contém `.deb`s e SDKs antigos: `boost_1_61_0.tar.xz` (não é mais necessário depois do item 1),
`flycapture2-2.5.3.4-amd64-pkg.tgz`, `imlib*.deb` e `zlib1g*.deb` de 2010. Nenhum Makefile do
`src/` referencia essa pasta (`grep -rl "ubuntu_packages" src/` não retorna nada) — é material
de instalação manual, não do build.

## g) GtkGLExt continua sem pacote

Ver `install/04`, seção 4.3. Nesta máquina ele já estava compilado e instalado em
`/usr/local` (veio da migração de outro repositório do LCAD). **Numa máquina nova é a
dependência mais provável de travar o build**, porque a GUI principal (`navigator_gui2`) e
mais ~20 módulos dependem dela e ela não existe em pacote nenhum.

## h) Módulos fora do `PACKAGES` com o padrão antigo de `Ptr` do PCL

O item 40 do `CHANGELOG.md` corrigiu, no `graphslam`, a construção de nuvens de pontos como
`boost::shared_ptr< pcl::PointCloud<T> >(new ...)` — desde o PCL 1.11 `pcl::PointCloud<T>::Ptr`
é `std::shared_ptr`, e não há conversão entre os dois.

O mesmo padrão continua em módulos que **não estão na lista `PACKAGES` do `src/Makefile`** e
portanto não são compilados hoje: `visual_graphslam`, `voslam`, `slam_icp` e `deep_vgl`. Não
foram alterados porque a correção não teria como ser verificada por um `make`. Quando algum
deles voltar para o `PACKAGES`, a correção é a mesma:

```bash
grep -rn "boost::shared_ptr" src/<modulo>
# boost::shared_ptr< pcl::PointCloud<T> >(new pcl::PointCloud<T>)  ->  pcl::PointCloud<T>::Ptr(new pcl::PointCloud<T>)
```

Atenção: em `voslam/voslam_rigid_transformation.h` o caso é diferente — é um campo
`boost::shared_ptr<pcl::Correspondences>`, cujo equivalente é `pcl::CorrespondencesPtr`.

## i) Binários pré-compilados em `bin/` vindos do 20.04

Cinco binários **não têm fonte no `carmen_lcad`** — vieram commitados, compilados numa máquina
Ubuntu 20.04 (commits `subindo binario do offroad`, `subindo lib offroad`). Por isso o `make`
da migração não os toca, e eles falham no `proccontrol` com `exited UNCLEANLY (code = 127)` —
que aqui não é "comando não encontrado", e sim o *loader* não resolvendo um soname.

Levantamento com `ldd`:

| Binário | Sonames que faltam | Situação |
|---|---|---|
| `route_planner` | `libgsl.so.23`, `libpython3.8.so.1.0` | ✅ resolvido pelo shim |
| `offroad_planner` | `libgsl.so.23` | ✅ resolvido pelo shim |
| `frenet_path_planner` | `libgsl.so.23`, `libpython3.8.so.1.0` | ✅ resolvido pelo shim |
| `task_manager` | `libpython3.8.so.1.0` + `libroscpp/rosconsole/rostime` | ❌ pendente (ROS1 Noetic) |
| `fastslam` | GSL 23, Python 3.8, **Boost 1.61**, **OpenCV 3.2** | ❌ pendente |

Esses sonames existiam no 20.04 porque o tutorial de instalação daquela época os instalava
explicitamente: `libgsl23` estava na lista do `apt-get install`, o Python do sistema do focal
**era** o 3.8, o tutorial mandava compilar OpenCV 3.2 do fonte em paralelo com a 4.5.5, e o
Boost 1.61 ia para `/usr/local/carmen_boost`. Nenhum deles tem pacote no 26.04
(`apt-cache policy libgsl23 libpython3.8` → sem candidato).

### O paliativo em uso: `lib/compat`

```bash
mkdir -p $CARMEN_HOME/lib/compat && cd $CARMEN_HOME/lib/compat
ln -sfn /usr/lib/x86_64-linux-gnu/libgsl.so.28.0.0      libgsl.so.23
ln -sfn /usr/lib/x86_64-linux-gnu/libpython3.14.so.1.0  libpython3.8.so.1.0
# no ~/.bashrc, dentro do bloco #CARMEN:
export LD_LIBRARY_PATH=$CARMEN_HOME/lib/compat:$LD_LIBRARY_PATH
```

O shim foi validado antes de ser adotado, não é chute:

- **todos** os símbolos exigidos existem nas libs novas (10 `gsl_*` do `route_planner`,
  13 do `offroad_planner`, 14 `Py*`) — nenhum faltando;
- não há *symbol versioning* nas referências indefinidas, então não há `@GLIBC_`-style
  para quebrar;
- com o shim, `proccontrol argos/process-argos-navigate.ini` sobe **sem nenhum
  `code = 127`**.

⚠️ **Símbolo presente não é ABI igual.** GSL 2.5→2.8 é baixo risco (as funções usadas são de
API opaca e estável). **Python 3.8→3.14 é aposta**: o `_Py_Dealloc` na lista de indefinidos
mostra que `Py_DECREF` foi inlinado, ou seja o binário mexe em `ob_refcnt` num offset fixo, e
objetos imortais (3.12+) passam a ser decrementados indevidamente. Atenua o risco o fato de
`Py_Initialize()` **não** ser chamado no startup (testado com `PYTHONHOME=/nao_existe`: nenhum
fatal error) — o Python ali é caminho preguiçoso. Se aparecer comportamento estranho no
`route_planner`, trocar o symlink por uma `libpython3.8` real extraída de um `.deb` do focal
(junto com a stdlib 3.8 e `PYTHONHOME`).

Há ainda uma deriva **silenciosa**, que o `ldd` não denuncia: o `offroad_planner` linka
`libcasadi.so` (soname sem versão). No 20.04 vinha do `pip3 install casadi` via `CASADI_DIR`;
no 26.04 resolve para o pacote do sistema, CasADi **3.7.0**. Ele também depende de
`libtrailer_nlp.so` (outra lib pré-compilada, em `$CARMEN_HOME/lib`), que resolve normalmente.

**A correção de verdade** é conseguir o fonte desses módulos com quem os mantém no LCAD e
recompilá-los no 26.04 — aí `lib/compat` inteiro pode ser apagado.

## j) Próxima fronteira: runtime

Compilar não é rodar. Com a árvore de pé, o próximo conjunto de problemas — completamente
diferente dos de compilação — aparece ao subir `central` → `param_daemon` → `proccontrol` com
um `process-*.ini` de `bin/`. O que esperar:

- `.ini` apontando para paths/versões antigas;
- `LD_LIBRARY_PATH` — atenção especial se a máquina tiver outro repositório do LCAD
  com libs de mesmo nome; ver `00_ambiente_e_bashrc.md`;
- permissões de device (LIDAR/CAN/câmera) e drivers que dependem de hardware;
- GUIs GTK2 em tela HiDPI/4K (o GTK2 não tem suporte a HiDPI — a interface sai
  "embaralhada"/minúscula; contorno é rodar com `GDK_SCALE`/resolução menor).
