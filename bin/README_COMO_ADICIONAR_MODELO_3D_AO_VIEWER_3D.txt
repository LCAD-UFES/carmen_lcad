# Adicionando novo modelo 3D ao viewer_3d

Aparentemente o viewer_3d ignora o eixo centro do .obj, e usa o centro do modelo.
A boa prática é criar uma pasta no bin com os arquivos do seu modelo (não suba o fonte, pode ser muito grande, salve no drive e coloque um readme na pasta com o link)
Após ter criado o modelo e exportado como .obj coloque o caminho no carmen-....ini do robô desejado (ex: carmen-ford-escape.ini, carmen-ecotech4.ini)
  Ex:
  carmodel_file_name     ecotech4_model/ecoTech4.obj

Ajuste o tamanho do modelo usando os parâmetros (tamanho em metros, usado no modelo):
carmodel_size_x		2.850
carmodel_size_y		1.30
carmodel_size_z		1.650

Aparentemente o Y e o Z são trocados, ou não influenciam na escala do modelo (Verificar codigo do viewer_3d).

Agora ajuste o centro do modelo para o eixo traseiro do robô. (Centro do Ackerman) Alterando os seguintes parâmetros:
carmodel_x		0.0
carmodel_y		0.0
carmodel_z		0.0
carmodel_roll	0.0
carmodel_pitch	0.0
carmodel_yaw	0.0

primeiro zere todos os valores para que o eixo vá para o centro do modelo,
Para garantir a visualização adequada zere tbm a posição da sensor_board, xsens_ e gps_nmea (guarde os valores reais para não perder):

sensor_board_1_x		0.0 
sensor_board_1_y		0.0
sensor_board_1_z		0.0 
sensor_board_1_yaw		0.0 
sensor_board_1_pitch	0.0 
sensor_board_1_roll		0.0
sensor_board_1_laser_id	0


 a partir daí coloque os valores de forma que o eixo (representado pelo GPSaxis) vá para o centro do eixo traseiro.
É possível, uma vez identificando a posição do centro do objeto, medir no programa de modelagem 3D a distancia até o eixo e alterar nos parametros.
Quanto maior o valor, o eixo irá se deslocar para trás.

Após o eixo estar corretamente ajustado, coloque os valores da sensor_board de volta e verifique se os raios do velodyne que batem no veículo, estão
batendo corretamente no modelo. Este mecanismo auxilia na visualização e ajuste de sensores, quanto mais próximo do modelo real, melhor.

# Textura modelos viewer_3d

Muito importante, o viewer_3d só aceita texturas apartir de imagens, no formato jpg.
O E a textura eh invertida verticalmente e horizontalmente.
quando for exportar seu modelo com textura, ele gerará um arquivo .obj e um arquivo .mtl o mtl é o arquivo que tem as texturas, ele pode carregar
a textura de um jpg com mesmo nome de quando foi salvo. Por exemplo, se a textura aplicada esta em um arquivo minha_textura.jpg e você pode
alterar essa textura (mantendo a localidade/contorno etc) e ele sempre caregará esse arquivo com esse nome minha_textura.jpg.

