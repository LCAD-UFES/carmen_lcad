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
<br>
OBS.: <br>
O arquivo <br>
config/basepos-20191003-20191003-5m-1m.txt, referente ao log_volta_da_ufes-20191003.txt (disponível no google drive de logs do LCAD)<br>
contém a lista de labels e respectivas imagens e poses, no seguinte formato<br>
<sub>image label x y z rx ry rz timestamp<br>
/dados/ufes/20191003/1570117889.210752.bb3.l.png 0 7757730.3562 -363561.284062 0.0 0.037827 0.023097 0.674785 1570117889.210752<br>
/dados/ufes/20191003/1570117928.424578.bb3.l.png 1 7757734.3402 -363558.144286 0.0 0.035305 -0.016079 0.677648 1570117928.424578<br>
/dados/ufes/20191003/1570117929.611112.bb3.l.png 2 7757738.44882 -363555.113682 0.0 0.037699 -0.030118 0.662518 1570117929.611112<br>
/dados/ufes/20191003/1570117930.610327.bb3.l.png 3 7757742.55146 -363552.010419 0.0 0.038893 -0.043297 0.661246 1570117930.610327<br>
/dados/ufes/20191003/1570117931.547136.bb3.l.png 4 7757746.7374 -363548.810127 0.0 0.046705 -0.050046 0.669928 1570117931.547136<br>
/dados/ufes/20191003/1570117932.421384.bb3.l.png 5 7757751.0108 -363545.509366 0.0 0.059163 -0.051509 0.672911 1570117932.421384<br>
/dados/ufes/20191003/1570117933.233292.bb3.l.png 6 7757755.21166 -363542.282578 0.0 0.0505 -0.050288 0.667493 1570117933.233292<br>
/dados/ufes/20191003/1570117933.982701.bb3.l.png 7 7757759.3385 -363539.104213 0.0 0.045324 -0.050641 0.660624 1570117933.982701<br>
/dados/ufes/20191003/1570117934.732098.bb3.l.png 8 7757763.63225 -363535.829442 0.0 0.041511 -0.050475 0.652415 1570117934.732098<br>
</sub>
