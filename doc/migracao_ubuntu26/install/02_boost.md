# 2. Boost

**Diferente do tutorial antigo: NÃO compile o Boost na mão e NÃO use o prefixo
`/usr/local/carmen_boost`.**

```bash
sudo apt-get install -y libboost-all-dev
```

Isso instala o Boost 1.90 (candidato no Ubuntu 26.04 "resolute") em `/usr/include/boost` e
`/usr/lib/x86_64-linux-gnu/libboost_*` — diretórios que o compilador e o linker já procuram
por padrão, sem precisar de `-I`/`-L` nenhum.

## O que foi corrigido no repo por causa disso

Os 28 `Makefile`s que apontavam para `-I/usr/local/carmen_boost/include` /
`-L/usr/local/carmen_boost/lib` (o Boost 1.61 compilado à mão do tutorial de 16.04/20.04)
tiveram essas flags removidas — ver `../CHANGELOG.md`, item 1. Sem elas, o build usa o Boost
do sistema.

Além do prefixo, três coisas do Boost antigo não existem mais e também foram corrigidas
(itens 2, 3 e 4 do `../CHANGELOG.md`):

| Flag antiga | Situação hoje | O que foi feito |
|---|---|---|
| `-lboost_thread-mt` | O sufixo `-mt` sumiu há mais de uma década; hoje é `libboost_thread.so` | Trocado por `-lboost_thread` |
| `-lboost_signals` | `Boost.Signals` (v1) foi **removido** do Boost; não existe pacote `libboost-signals-dev` | Flag removida; o único uso real da API (em `sharedlib/libtf`) foi portado para `Boost.Signals2` (header-only) |
| `-lboost_system` | `Boost.System` é header-only; o pacote do 26.04 não gera mais a `.so` de compatibilidade | Flag removida |

## Se aparecer erro de Boost mesmo assim

- `cannot find -lboost_<algo>`: sobrou algum Makefile que a varredura não pegou. Reportar
  qual módulo — a correção é a mesma dos itens acima.
- `#warning "Boost.Math requires C++14"` seguido de erro (`'is_final' has not been declared
  in 'std'`): é um módulo compilando em `-std=c++11` que inclui `tf.h` (o `libtf` usa
  `boost/math`). Ver `../CHANGELOG.md`, item 10 — a correção é subir esse módulo para
  `-std=c++14`.

## Por que o tutorial antigo mandava compilar o Boost na mão

Duas razões, ambas superadas:
1. **Conflito com o Boost do ROS** em máquinas com ROS Noetic — hoje isso só afetaria as
   bridges ROS (`src/ros1_sc_lio_sam`, `src/ros1-lt-mapper`), que são builds `catkin`
   separados, fora do `make` do `src/`.
2. **Nomes `-mt`** — convenção morta, já corrigida no repo.

Próximo passo: `03_opencv_pcl_g2o_dlib_eigen.md`.
