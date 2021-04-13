Rode o central como abaixo:
./central -Lmtrnds -fsample_central.log

Rode seu process (não por muito tempo, pois o comando acima escreve todas as mensagens no arquivo sample_central.log).

Rode o comando:
./central_log_view sample_central.log -mod 'localize*:*' 'map*:*' -imod '*IPC*' 'param_daemon*' -imsg carmen_param_query_version -s co

O comando acima mostra todas as mensagens no log oriundas os modulos localize* e map* e destinadas a qualquer modulo (o asterisco depois dos ":").
Os demais parametros removem mensagens de servico do IPC para tornar a saida do programa central_log_view mais legivel.
(Ver mais detalhes sobre o comando central_log_view em src/utilities/central_log_view/readme.md).

