# ArUco Position Tracker

ArUco é uma biblioteca OpenSource para detectar marcadores fiduciais em imagens.

Além disso, se a câmera estiver calibrada, você pode estimar a pose da câmera em relação aos marcadores.

## Instalação Aruco 3.1.12

Download https://sourceforge.net/projects/aruco/, e unzip em `$ASTRO_HOME/sharedlib`.

Então, instale

```bash
mkdir build
cd build
cmake ..
make
sudo make install
```

Note que é necessário instalar o OpenCV 4, conforme descrito no [tutorial de instalação Carmen LCAD](https://github.com/LCAD-UFES/astro_lcad/wiki/Installing-Carmen-on-Ubuntu-20.04#instalar-opencv-455--).


Você pode ler mais sobre a técnica na [documentação do ArUco](ArUco_documentation.pdf).
