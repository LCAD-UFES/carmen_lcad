# link com as imagens dos logs da IARA - Volta da UFES
https://drive.google.com/drive/folders/1tqRKGO3DtW1yreoxYeD9Ssc3Ip8fxaXC?usp=sharing

20160825<br>
20160825-01<br>
20160825-02<br>
20160830<br>
20160902<br>
20161021<br>
20170119<br>
20171205<br>
20180112<br>
20180112-02<br>
20191003<br>
<br>
escolha um dos logs e salve em um local acessível em seu computador.<br>
ex.:  /dados/ufes/20161021<br>
<br>
!!!! lembre-se de executar o comando "git pull" para ter sempre a versão mais atual do carmen e submódulos !!!!!<br>
<br>
execute os seguintes comandos para gerar a lista de imagens: (altere o caminho de acordo com seu caso)<br>
cd /dados/ufes/<br>
find `pwd`/20161021/ -name \*.l.png > imagens.txt<br>
mv imagens.txt $CARMEN_HOME/src/localize_neural2/config/train.txt<br>
<br>
Para compilar basta executar o comando make da seguinte forma:<br>
cd $CARMEN_HOME/src/localize_neural2/<br>
make<br>
<br>
para testar execute:<br>
./localize_neural2<br>

