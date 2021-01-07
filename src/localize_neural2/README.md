# link com as imagens dos logs da IARA - Volta da UFES
https://drive.google.com/drive/folders/1tqRKGO3DtW1yreoxYeD9Ssc3Ip8fxaXC?usp=sharing

20160825
20160825-01
20160825-02
20160830
20160902
20161021
20170119
20171205
20180112
20180112-02
20191003

escolha um dos logs e salve em um local acessível em seu computador.
ex.:  /dados/ufes/20161021

!!!! lembre-se de executar o comando "git pull" para ter sempre a versão mais atual do carmen e submódulos !!!!!

execute os seguintes comandos para gerar a lista de imagens: (altere o caminho de acordo com seu caso)
cd /dados/ufes/
find `pwd`/20161021/ -name \*.l.png > imagens.txt
mv imagens.txt $CARMEN_HOME/src/localize_neural2/config/train.txt

Para compilar basta executar o comando make da seguinte forma:
cd $CARMEN_HOME/src/localize_neural2/
make

para testar execute:
./localize_neural2
