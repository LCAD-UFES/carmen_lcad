#include "visualControl.h"
#include "time.h"

void drawRect(CvRect rect, IplImage** im, CvScalar color);
CvPoint2D32f convertRect2centerPoint(CvRect rect, CvScalar color);

VisualControl::VisualControl(int width, int height)
:v1()
,vert1(cvSize(width,height))
{
	vmin = 110, vmax = 256, smin = 180; //valores para definir a mask do HSV.
	imagePersHSV = cvCreateImage(cvSize(width,height), 8, 3);
	mask = cvCreateImage(cvSize(width,height), 8, 1);
	pt_nc = cvPoint2D32f(0,0);
	pt = cvPoint2D32f(0,0);
}

void VisualControl::update(IplImage *im)
{
	vert1.track(im,mask);
	pt = (vert1.getBox()).center;
	pt_nc = (vert1.getBox()).center;
	v1.estimaPos();
	v1.correctPos(&pt.x,&pt.y);
}

int VisualControl::startVisualControl(const IplImage* im, const CvRect *rect){

	CvScalar color = CV_RGB( 255, 0, 0 );

	if (rect)
		vert1.setRect(rect->x, rect->y, rect->width, rect->height);

	//Calcula o Histograma dos quadrados sobre os quais se aplicar� o Tracking.
	if(vert1.calcHist(im)){
		cout<<"Histograma n�o Calculado";
		return -1;
	}

	int _vmin = vmin, _vmax = vmax;

	cvCvtColor( im, imagePersHSV, CV_BGR2HSV );
	cvInRangeS( imagePersHSV, cvScalar(0,smin,MIN(_vmin,_vmax),0),cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

	//Para se garantir uma estabilidade maior do centro dos quadrados aplica-se uma primeira rodada de Tracking.
	for(int u = 0; u<5; u++){

		vert1.track(imagePersHSV,mask);

	}

	//Faz-se um track para incializar o filtro de Kalman com os valores iniciais para as posi��es dos v�rtices.
	vert1.track(imagePersHSV,mask);
	v1.startKalmanPosFilter((vert1.getBox()).center.x, (vert1.getBox()).center.y);
	
	//Recalcula os vertices do retangulo e os armazena
	update(imagePersHSV);

	return 0;
}

int VisualControl::trackVisualControl(const IplImage* im ){

	CvScalar color1 = CV_RGB( 255, 0, 0 );
	CvScalar color2 = CV_RGB( 50, 255, 50 );
	CvScalar color3 = CV_RGB( 50, 50, 255 );

#ifdef DEBUG
	cvNamedWindow("Imagem", CV_WINDOW_AUTOSIZE);
#endif
	int _vmin = vmin, _vmax = vmax;

	cvCvtColor( im, imagePersHSV, CV_BGR2HSV );
	cvInRangeS( imagePersHSV, cvScalar(0,smin,MIN(_vmin,_vmax),0),cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

	//Recalcula os vertices do retangulo e os armazena
	update(imagePersHSV);

	//Imprime os blobs do tracking e do retangulo de interesse para ser seguido----
	drawRect(vert1.getRect(),(IplImage**)&im,color2);
	cvCircle((IplImage*)im,cvPoint(pt.x, pt.y),3,color3);
	cvCircle((IplImage*)im,cvPoint(pt_nc.x, pt_nc.y),3,color1);

#ifdef DEBUG
	cvShowImage("Imagem", im);
	cvWaitKey(100);
#endif

	return 0;

}

VisualControl::~VisualControl(){

	cvReleaseImage(&imagePersHSV);
}

CvPoint2D32f convertRect2centerPoint(CvRect rect){

	CvPoint2D32f pt = cvPoint2D32f(0,0);
	pt.x = (float) rect.x;
	pt.y = (float) rect.y;
	pt.x = (float) (rect.x + rect.width);
	pt.y = (float) (rect.y + rect.height);
	return pt;
}

void drawRect(CvRect rect, IplImage** im, CvScalar color){

	CvPoint2D32f pt1;
	CvPoint2D32f pt2;

	pt1.x = (float) rect.x;
	pt1.y = (float) rect.y;
	pt2.x = (float) rect.x + rect.width;
	pt2.y = (float) rect.y + rect.height;
	cvRectangle(*im,cvPointFrom32f(pt1),cvPointFrom32f(pt2),color,2);
}

