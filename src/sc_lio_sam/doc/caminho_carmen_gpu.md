# Caminho definitivo e RViz2 em GPU

Em 2026-11-03, a pedido do usuário, o workspace foi movido de
`~/astro/src/sc_lio_sam_ros2` para `~/carmen_lcad/src/sc_lio_sam_ros2`.
Os 23 processos de IARA e ARGOS foram atualizados. Houve compilação limpa
no destino; caches e artefatos da instalação anterior foram guardados em
`/tmp/sc-lio-astro-relocation-bqaokles`.

Os dois playbacks usam `rviz_software_rendering:=false`. O RViz2 carregou
OpenGL da NVIDIA, e sua execução na GPU foi confirmada por `nvidia-smi`.
A opção de renderização por software permanece disponível apenas para
uso explícito de diagnóstico. A opção CUDA do SLAM não foi alterada.

## Verificação dos erros ao encerrar

Os testes monitoram os processos durante o playback e separam a fase de
operação da fase posterior ao SIGINT. Todos os nodos do SLAM e o RViz2
permaneceram ativos durante os trechos reproduzidos. Os mapas foram salvos
com pontos não vazios, e o mapper encerrou normalmente.

O SIGSEGV (código -11) observado exclusivamente no RViz2 após o SIGINT não
impediu a gravação do mapa. O mapa da IARA dessa execução foi recarregado e
publicado para assinante tardio, com encerramento em relógio pausado e hashes
dos arquivos inalterados. A causa raiz da falha gráfica não foi estabelecida.

Esse erro específico ao fechar pode ser tolerado para o fluxo de mapeamento
validado. Isso não se aplica a falhas durante a operação, a outros processos,
ou a erros de gravação do mapa. Os logs originais continuam expondo a falha;
ela não é suprimida no launch nem convertida silenciosamente em sucesso.

## Resultados

- IARA: 981 poses; `/tmp/proccontrol-iara-_bsw0rya`.
  RViz2 apresentou SIGSEGV somente após SIGINT; mapa e mapper OK.
- ARGOS: 1187 poses; `/tmp/proccontrol-argos-vxfxzu1u`.
  Encerramento limpo, incluindo RViz2.

Foram usados os processos indicados, com segundos 30–90 de cada log,
IPC/ROS isolados, saída de mapas em `/tmp` e redirecionamento dos logs.
Os mapas configurados em `/dados` não foram sobrescritos. Isso não equivale
à validação integral dos logs nem à inspeção visual de toda a trajetória.

Para reproduzir a verificação específica desta máquina:

```bash
cd ~/carmen_lcad/src/sc_lio_sam_ros2
python3 tests/proccontrol_playback.py iara --require-nvidia-rendering --allow-rviz-shutdown-crash
python3 tests/proccontrol_playback.py argos --require-nvidia-rendering --allow-rviz-shutdown-crash
```

Sem `--allow-rviz-shutdown-crash`, o teste continua estrito. Com essa opção,
somente o código -11 do RViz2 posterior ao SIGINT pode ser aceito, com warning,
e apenas se a operação, o mapper e os arquivos de mapa passarem nas verificações.
