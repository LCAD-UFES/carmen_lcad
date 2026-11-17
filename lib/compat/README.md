# lib/compat — shims de soname para binários pré-compilados

Estes symlinks existem **só** para os binários de `bin/` que vieram commitados no repo,
compilados no Ubuntu 20.04, e cujo fonte não está no `carmen_lcad`:
`route_planner`, `offroad_planner`, `frenet_path_planner`.

Eles pedem sonames que não existem mais no Ubuntu 26.04:

| soname pedido | de onde vinha no 20.04 | apontado aqui para |
|---|---|---|
| `libgsl.so.23` | pacote `libgsl23` (na lista do tutorial do 20.04) | `libgsl.so.28.0.0` |
| `libpython3.8.so.1.0` | Python do sistema do focal era o 3.8 | `libpython3.14.so.1.0` |

Não há pacote `libgsl23` nem `libpython3.8` no 26.04 (`apt-cache policy` → sem candidato).

**Isto é paliativo.** A correção de verdade é recompilar esses três módulos no 26.04, quando o
fonte deles estiver disponível — aí este diretório inteiro pode ser apagado.

Ver `doc/migracao_ubuntu26/install/05_pendencias_conhecidas.md`, seção j).

Para recriar numa máquina nova:

```bash
mkdir -p $CARMEN_HOME/lib/compat && cd $CARMEN_HOME/lib/compat
ln -sfn /usr/lib/x86_64-linux-gnu/libgsl.so.28.0.0      libgsl.so.23
ln -sfn /usr/lib/x86_64-linux-gnu/libpython3.14.so.1.0  libpython3.8.so.1.0
```
