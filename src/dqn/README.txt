
0) Instale o carmen com OpenCV
1) Instale a caffe
2) Faca os exports abaixo com os caminhos relativos a sua instalacao da caffe (adicione no .bashrc por conveniencia):

# diretorio da caffe (CORRIJA PARA O PATH DA SUA MAQUINA)
export CAFFE_HOME=/home/filipe/workspace/caffe/
# diretorio onde esta o executavel da caffe
export PATH=$PATH:$CAFFE_HOME/build/install/bin
# diretorio onde esta a libcaffe.so
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$CAFFE_HOME/build/install/lib

3) 

