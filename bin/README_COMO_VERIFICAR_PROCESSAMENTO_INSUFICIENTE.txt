O Carmen foi criado para ser capaz de funcionar mesmo com processamento insuficiente, contudo ele é muito melhor se for usado na capacidade máxima, e uma forma de verificar se não está havendo prejuízos por falta de processamento é verificar se está havendo perda de mensagens. Para isso, vá até o seu arquivo de parametros e coloque para o logger salvar a mensagem de globalpos:

logger_localize off -----> logger_localize	on

em seguida, rode o sistema por um pouco de tempo, e salve um log do que ocorreu, rodando em um terminal separado:

./logger /dados/log_teste.txt

posteriormente, utilize o código timestamp_graph para gerar um gráfico mostrando as publicações da mensagem:

./timestamp_graph GLOBALPOS_ACK /dados/log_teste.txt

Idealmente, o gráfico gerado será uma linha reta, que funcione na frequência do lidar (muitas vezes 20Hz), e é possível verificar a frequência contando quantos pontos tem em um segundo, dando zoom e habilitando o grid para facilitar, e caso a frequência seja menor, está com processamento insuficiente.
