www.lucidarme.me OpenGL application for mpu-9250 
More info at http://www.lucidarme.me/?p=5057

Install GLUT for OpenGL :

> sudo apt-get install freeglut3 freeglut3-dev



::: Option 1 :::

Open the project with Qt Creator (succesfully compiled on Qt Creator 3.0.1 / Qt 5.2.1 / GCC 4.8.2, 64 bit), clean, build and run. 



::: Option 2 :::

The makefile is also provided you can compile the application with :

> make clean ; make

Note that the application needs the rights on the USB port where the arduino / mpu-9250 is connected. 



