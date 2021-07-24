##########################################################################################
# Atenção! Ainda esta em teste, parece haver algum problema de sicronização
# (o código parece não sicronizar/corrigir atraso na mensagem da odometria e a odometria visual é mais lenta que a normal)
###############################################################################################
Para criar um mapa usando a odometria visual, é necesário que o visual_odometry publique a mensagem carmen_robot_ackerman_velocity_message que eh a verdadeira odometria (v e phi) vinda do hardware.
Para isso é necessário que quando o log for gravado gravar as imagens para depois gerar a odometria visual, ou gravar a odometria visual diretamente em vez da do carro.
Para ativar a visual odometry em vez da odometria do carro, altere as seguintes flags no carmen.ini 

	visual_odometry_publish on
	robot_publish_odometry	off

Rode a visual odometry no seu process de log (detalhes no README)
Lembre-se que os parâmetros da imagem da camera devem estar corretos e dos parametros do stereo também. 
Se estiver usando a mensagem da Bumblebee_basic1 os parametros do stereo1 devem ser equivalentes
	./visual_odometry2 1  -publish_base_ackerman_odometry off -compare_odometries off

Quando a mensagem estiver gravada no seu log, utilize o modo comum de fazer mapas.
README_COMO_FAZER_MAPAS_USANDO_O_GRAPHSLAM.txt

Detalhes dos modulos em:
$CARMEN_HOME/src/visual_odometry_package2/README_Alberto.txt

Caso a odometria visual não tenha sido gravada. Isso também pode ser feito da seguinte forma:
Rode o playback com a odometria visual ligada usando os parametros citados anteriormente, em outro terminal rode o logger e crie um log que conterá a odometria visual.
Com isso seu log terá toda informação para seguir com o modo comum de fazer mapas.
