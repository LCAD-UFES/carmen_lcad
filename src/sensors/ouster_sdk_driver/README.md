## Ouster from ouster C++ SDK version 0.4.1

Esse módulo usa o SDK mais atual da ouster (09/08/2022) e está pronto para rodar todos os sensores pegando os parâmetros 
do sensor automaticamente (sem necessidade de gravar no carmen-ini ou fixar para sempre um numero de laser)

TODO:

- Os raios do ouster tem azimuth offset diferente em relação ao zero, ou seja, os 32 raios não estão alinhados
dessa forma, para gerar uma nuvem de pontos mais correta possivel, seria necessário que além de usar o Vertical angle, usar o azimuth angle na hora de converter os pontos
para carteziano.
Como a mensagem Variable Partial scan assume que os raios de cada shot esteja alinhado, a correção é feita apenas cnsiderando o offset do primeiro raio.
Isso só é útil para o viewer 3D e talvez para módulos que usem o velodyne_camera_calibration para converter a posição dos pontos corretamente, para a evidencia de obstáculo, por ser pouca a diferença
traria mais problemas do que ganho (Opnião do Vinicius)

- Publicar imagem gerada pela reflexão do sensor, para testarmos detecção na imagem do lidar

###Instalação

Baixe o SDK usado -> link
baixe as dependencias:

instale no packages carmen
	build
	cmake
	make
	sudo make instal

compile o modulo carmen