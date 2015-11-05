//Essa Classe tem por objetivo realizar o tracking de objetos... No presente trabalho mais espec�ficamente, ser� realizado o 
//tracking de quatro quadrados cujo os centros ser�o considerados os v�rtices de um quadrado maior... Esses v�rtices ser�o a
//refer�ncia para o sistema de controle. Cada v�rtice ter� um objeto dessa classe associado a ele...

#ifndef VERT_RETANGULO_H
#define VERT_RETANGULO_H

#ifndef VISUAL_CONTROL_H 
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <iostream>
#endif

using namespace std;

class VertRetangulo{

private:

	//Tamanho da imagem a ser utilizada para realizar o tracking.
	CvSize src;
	CvRect rect;
	IplImage* hsv;// N�o ser� utilizada dentro do track, tendo em vista que, a convrs�o de RGB para HSV 
				  //dentro do object to track demanda tempo, ser� feito fora da fun��o track.
	
	//Usado na inicializa��o do Histograma.
	int histSize[1];
	float hRanges[2];
	float* ranges[1];

	//Usado no c�lculo do histograma e no camshift.
	IplImage* hPlane;
	IplImage* sPlane;
	IplImage* vPlane;
	IplImage* planes[1];
	CvHistogram* hist;

	//usado na fun��o camshift e backproject.
	CvConnectedComp* comp;
	CvBox2D* box;
	IplImage* backProject;
	IplImage* mask;//N�o ser� mais utilzado dentro do track tendo emvista que pode tomar tempo de 
				   //processamento. Sendo assim, esse processamento ser� feito fora da fun��o track.
				   //O mask ser� criado dentro da fun��o VisualControl::SpeedControl();

	int vmin, vmax, smin; //valores para definir a mask do HSV. N�o est� sendo usado por enquanto.
	int _vmin, _vmax;

public:

	VertRetangulo(CvSize sz);
	void setRect(int x, int y, int width, int height);
	CvRect getRect();
	CvBox2D getBox();
	int calcHist(const IplImage* frame);
	void updateHist(const IplImage* frame);
	void track(IplImage* frameHSV,IplImage* frameMask);
	int rectSelect(const IplImage* frame);
	~VertRetangulo();

};

#endif
