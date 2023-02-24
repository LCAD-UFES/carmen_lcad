# param_daemon

O módulo **param_daemon** é responsável por carregar, gerir e prover dinamicamente o conteúdo das variáveis paramétricas de todos os módulos do sistema. Portanto, ele deve ser o primeiro módulo a ser executado, logo após o IPC `central` iniciar.

Os seguintes argumentos podem ser utilizados na linha de comando do `param_daemon`:

```
 -a			alphabetize
 -r <robot_name>	robot
 -l <log_file>		log

```


# param_edit

Uma vez que o módulo `param_deamon` esteja em execução, o programa **param_edit** pode ser utilizado para visualizar o conteúdo das variáveis paramétricas, modificar dinamicamente os seus valores, e opcionalmente salvar o conteúdo modificado no mesmo arquivo ini ou em novo arquivo. 


### Como modificar dinamicamente as variáveis paramétricas do arquivo ini (sem interromper a execução dos módulos do sistema):

1. Verifique no programa do módulo desejado, todas as chamadas à função "carmen_param_install_params". 
   Geralmente isto ocorre dentro da função "read_parameters" ou alguma semelhante.
   O terceiro argumento da função "carmen_param_install_params" é um vetor de struct "carmen_param_t".
   Exemplo:
      carmen_param_t param_list[] =
      {
         {(char *) "mapper", (char *) "velodyne",       CARMEN_PARAM_ONOFF,  &sensors_params[0].alive,                  0, NULL},
         {(char *) "mapper", (char *) "velodyne_locc",  CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_occ,  0, NULL},
         {(char *) "mapper", (char *) "velodyne_lfree", CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_free, 0, NULL},
         {(char *) "mapper", (char *) "velodyne_l0",    CARMEN_PARAM_DOUBLE, &sensors_params[0].log_odds.log_odds_l0,   0, NULL},
         {(char *) "mapper", (char *) "velodyne_unexpeted_delta_range_sigma", CARMEN_PARAM_DOUBLE, &sensors_params[0].unexpeted_delta_range_sigma, 0, NULL},
      };
   Cada elemento deste vetor corresponde a um parâmetro que será lido do carmen.ini. 
   Juntando-se o conteúdo das duas primeiras colunas, temos o nome do parâmetro (p.ex.: "mapper_velodyne").
   NOTA: O nome do módulo que aparece na primeira coluna não pode conter sublinhado (_). Se contiver, transfira o restante do nome para a segunda coluna.
         Por exemplo, em vez de:
         {(char *) "behavior_selector", (char *) "change_goal_distance", CARMEN_PARAM_DOUBLE, &change_goal_distance, 0, NULL},
	 Utilize:
         {(char *) "behavior", (char *) "selector_change_goal_distance", CARMEN_PARAM_DOUBLE, &change_goal_distance, 0, NULL},
   A terceira coluna indica o tipo de valor que será lido: INT (inteiro), DOUBLE (ponto flutuante), ONOFF (1 ou 0 inteiro), STR (caracteres), FILE ou DIR.
   A quarta coluna indica o ponteiro da variável que receberá o valor lido. Preferencialmente deve-se utilizar variável global.
   A quinta coluna indica se a variável será mod	ificada dinamicamente no programa. Coloque valor 1 caso queira habilitar isto.
   A sexta coluna opcionalmente indica uma função "handler" que será acionada quando o IPC central receber a modificação dinâmica do parâmetro.
   A função "handler" é necessária quando há algum procedimento complementar de inicialização logo após a chamada à função "carmen_param_install_params".
   Nos demais casos, basta colocar o valor 1 na quinta coluna e NULL na sexta coluna e as modificações dinâmicas serão ativadas.  
   Por exemplo, suponha que um parâmetro esteja expresso no carmen.ini em graus (0 a 360) mas o programa opere com valores em radianos.
   Neste caso deve-se criar um handler que transforme o parâmetro lido, de graus para radianos.
   Como regra geral, toda modificação de conteúdo de parâmetros que é feita imediatamente após a chamada à função "carmen_param_install_params", 
   deve ser tratada em handlers, caso os parâmetros possam ser modificados dinamicamente.
   Um exemplo de implementação de handler está nas funções "get_alive_sensors" e "sensors_params_handler" no programa "mapper_main.cpp".
   
2. Com o IPC central e o módulo "param_daemon" em execução, para modificar dinamicamente os parâmetros de carmen.ini, execute o programa "param_edit".
   Este programa abre uma janela com o título "Param Editor".
   No quadro à esquerda da janela, selecione o módulo desejado.
   NOTA: Se não aparecer nada no quadro à direita da janela, clique no menu superior "View > Expert Params"
   No quadro à direita da janela, selecione o parâmetro desejado e modifique o seu valor, seguido pela tecla "Enter".
   Aparecerá a mensagem no rodapé da janela: "Saving parameters... done".
   Ao final, caso deseje salvar permanentemente em arquivo os dados que foram alterados dinamicamente, clique no menu superior "File > Save ini".
   Aparecerá a mensagem no rodapé da janela: "Saving <arquivo.ini>... done".
   