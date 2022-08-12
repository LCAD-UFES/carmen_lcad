Para criar um rddf roda o programa rddf_build enquanto faz playback de um log.

>> ./rddf_build -rddf rddf.kml

OBS: Para corrigir o fechamento de loop, abra o arquivo kml com o google-earth e apague pontos do inicio ou do final de forma que o final e o inicio do caminho fiquem o mais proximos quanto for possivel. Em seguida, abra o arquivo kml e copie a regiao a primeira tag <Placemark> </Placemark> apos a ultima tag <Placemark> </Placemark>. Ao fazer isso, o ultimo waypoint sera igual ao primeiro, criando um loop perfeito. Para que esse ultimo ponto que voce copiou seja colocado no final no indice de timestamp (para simular que apos terminar a volta, o carro voltou ao inicio), faca com que o timestamp do placemark que voce colou no final do arquivo seja igual ao anterior mais 1.

Para rodar o rddf, use o programa rddf_play.

>> ./rddf_play -rddf rddf.kml

Para adicionar annotations ao RNDF abra o arquivo KML com o Google Earth, clique no placemark desejado com o botao direito e va em propriedades. Na descricao, adicione o texto que se refere a sua annotation. Os textos possiveis sao:
- human intervention: pontos de parada para intervencao humana, como catracas, etc.
- speed bumper: quebra-molas.

OBS: NAO USE ASPAS!!

Anotações que usam o último campo da lista de anotações:

RDDF_ANNOTATION_TYPE_PEDESTRIAN_TRACK		3	0	3.016		7756880.63	-364044.30	7.0

Nessa anotação, o último valor (7.0) define o raio da cobertura da faixa de pedestre.


RDDF_ANNOTATION_TYPE_TRAFFIC_LIGHT		8	0	-2.312832	7757498.447006	-364158.309644	6.671012

Nessa anotação, o último valor (6.0) é a altura do semáfaro.