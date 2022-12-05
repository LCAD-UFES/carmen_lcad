# ArUco

ArUco é uma biblioteca OpenSource para detectar marcadores fiduciais em imagens.

Além disso, se a câmera estiver calibrada, você pode estimar a pose da câmera em relação aos marcadores.


## Instalação Aruco 3.1.12

Download https://sourceforge.net/projects/aruco/, e unzip em `$CARMEN_HOME/sharedlib`.

Então, instale

```bash
mkdir build
cd build
cmake -DOpenCV_DIR=~/packages_carmen/opencv-4.5.5/build ..
make
sudo make install
```

Note que é necessário instalar o OpenCV 4, conforme descrito no [tutorial de instalação Carmen LCAD](https://github.com/LCAD-UFES/carmen_lcad/wiki/Installing-Carmen-on-Ubuntu-20.04#instalar-opencv-455--).



## Uso

Este módulo publica os vetores de translação e rotação dos marcadores ArUco **em relação à câmera**.

Você pode ler mais sobre a técnica na [documentação do ArUco](ArUco_Library_Documentation.pdf).


## Auxiliares

* `aruco_test`: printa no terminal o que está sendo publicado pelo módulo.
* `create_board`: cria MarkerMapPose ArUco para detecção mais precisa:

> `./run <XSize> <YSize> <out.png> <out.yml> <factor_quality>`
> `./run 6 1 out.png out.yml 10`\
> `factor_quality` é um fator que aumenta a qualidade da imagem gerada, necessário para uma boa impressão.