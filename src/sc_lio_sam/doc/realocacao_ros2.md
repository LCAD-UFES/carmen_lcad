> Histórico: o caminho e a renderização foram atualizados posteriormente.
> Consulte [Caminho definitivo e RViz2 em GPU](caminho_carmen_gpu.md).

# Realocação do workspace ROS2

Em 2026-11-03, o workspace nativo foi transferido para
`/home/eduardopia/astro/src/sc_lio_sam_ros2`.
A documentação de execução e compilação atual está no README desse diretório.
O CARMEN, suas bibliotecas, parâmetros e processos continuam em `~/carmen_lcad`.

Foram atualizados 23 processos em `data/iara/process`, `bin/argos` e
`data/argos/process`, preservando variáveis e estados de inicialização.
Os nove processos antigos de `data/argos/process` também foram convertidos
para launches nativos e parâmetros ROS2 das ferramentas auxiliares.

As pontes e os testes diferenciais da transição ROS1 foram arquivados em
`doc/ros1_transition_reference.tar.gz` antes de serem removidos do workspace
ativo. A cópia contém os fontes, scripts, README e testes anteriores à limpeza.
Os processos anteriores estão em `doc/processes_before_relocation`.
Os artefatos antigos de compilação estão em `relocation_build_backup`, fora
do workspace novo e ignorados pela descoberta de pacotes catkin/colcon.

Os dois processos pedidos foram executados por proccontrol antes da mudança,
com IPC/ROS isolados, logs em `/tmp`, RViz2 e mapa temporário sobrescrito via
`-SET SC_LIO_SAM_MAP_PATH=...`. Foram reproduzidos os segundos 30–90:

- IARA: 992 poses; `/tmp/proccontrol-iara-qkm5c8d_`.
- ARGOS: 1191 poses; `/tmp/proccontrol-argos-y05b3ebn`.

Ambos salvaram mapas e encerraram os nodos ROS2 normalmente. Esses ensaios
não equivalem à reprodução integral dos logs nem à validação visual completa
dos mapas. Os testes reproduzíveis estão em `tests/proccontrol_playback.py`
no novo workspace.

## Verificações da instalação final

A compilação limpa no novo diretório incluiu CUDA. Foi corrigido um problema
preexistente em `Scancontext.cpp`: `xy2theta` podia terminar sem retorno e
calcular `0/0` na origem. O cálculo agora usa `atan2`, define a origem como
zero e descarta pontos não finitos antes da indexação do descritor.
O teste `run_scancontext_geometry_test.py` exercita o objeto compilado.
A recompilação final terminou sem avisos.

Os 14 executáveis nativos tiveram suas dependências verificadas no ambiente
ROS2, sem bibliotecas do Noetic. A instalação não contém scripts ou launches
de ponte ROS1. A sintaxe dos scripts e as referências nos 23 processos foram
verificadas.

O teste de carregamento com configurações IARA e ARGOS passou: mapa global
para assinante tardio sem `/clock`, encerramento com relógio parado e hashes
dos arquivos do mapa inalterados. Logs: `/tmp/mapping-localization-2uhuclbu`.

Na primeira repetição final do ARGOS, o SLAM salvou o mapa normalmente, mas
RViz2 falhou no encerramento. Aumentar o timeout e retirar o tema GTK isoladamente não
resolveram; o timeout foi restaurado ao padrão. O teste isolado com Mesa por
software passou em três ciclos de abertura/encerramento, enquanto o caminho
NVIDIA apresentou SIGSEGV. Isso isola a falha no caminho gráfico, mas não
identifica sua causa raiz no driver.

O launch oferece `rviz_software_rendering` (default false). Os dois processos
de playback pedidos habilitam essa opção; ela define `LIBGL_ALWAYS_SOFTWARE=1`
e `__GLX_VENDOR_LIBRARY_NAME=mesa`, com tema Qt `generic` e estilo `Fusion`,
somente no RViz2. Mesa sem retirar GTK passou no ARGOS, mas ainda falhou
no encerramento da IARA; por isso a opção combina os dois ajustes. CUDA no SLAM não é afetada.
Os demais processos preservam o renderizador padrão. O encerramento mantém
o timeout normal; a falha não é escondida nem ignorada pelo teste.

## Resultado dos playbacks finais

- IARA: 709 poses; mapa salvo e encerramento limpo, incluindo RViz2.
  Logs e mapa: `/tmp/proccontrol-iara-fq2c57rf`.
- ARGOS: 1192 poses; mapa salvo e encerramento limpo, incluindo RViz2.
  Logs e mapa: `/tmp/proccontrol-argos-6xtl7znt`.

Os processos mantêm seus nomes e comandos habituais em `~/carmen_lcad/bin`.
A única opção adicional nos dois playbacks é a renderização segura do RViz2.
A mudança não é uma validação integral dos logs, da qualidade dos mapas ou
da execução de CUDA em todos os cenários.
