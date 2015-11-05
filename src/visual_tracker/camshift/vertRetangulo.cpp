#include "vertRetangulo.h"
#include "time.h"
//Fun��es Internas ao Arquivo
void mouseCallBack(int event, int x, int y, int flags, void* param);

//Fun��es da Classe
VertRetangulo::VertRetangulo(CvSize sz){


	vmin = 80, vmax = 256, smin = 100; //valores para definir a mask do HSV. N�o est� sendo usado por enquanto.
	_vmin = vmin, _vmax = vmax;

	int hBins = 181;
	histSize[0] = hBins;
	hRanges[0] = 0;
	hRanges[1] = 180;
	ranges[0] = hRanges;
	src = sz;

	rect = cvRect( -1, -1, 0, 0);
	backProject = cvCreateImage(src, 8, 1);
	comp = new (CvConnectedComp);
	box = new (CvBox2D);
	hist = cvCreateHist(1,histSize,CV_HIST_ARRAY,ranges,1);

	hsv = cvCreateImage(src, 8, 3);		//A convers�o do HSV, bem como a cria��o do Mask n�o � mais feita em sua totalidade
	mask = cvCreateImage( src, 8, 1 );	//dentro desta classe. Ela � feita apenas para o calculo do histograma, pois na hora
										//da realiza��o do track, voc� deve passar como par�metro o hsv e o mask. Isso foi											//feito para que se reduza o tempo de processamento do programa, pois antes o mask e										//o HSV da imagem era feito repetidamente para cada objeto da classe vertRetangulo. 

	hPlane = cvCreateImage(src, 8, 1 );
	sPlane = cvCreateImage(src, 8, 1 );
	vPlane = cvCreateImage(src, 8, 1 );
	planes[0] = hPlane;

}

void VertRetangulo::setRect(int x, int y, int width, int height){
	rect = cvRect(x, y, width, height);
}

CvRect VertRetangulo::getRect(){
	return rect;
}

CvBox2D VertRetangulo::getBox(){
	CvBox2D boxAux;
	boxAux.angle = box->angle;
	boxAux.center = box->center;
	boxAux.size = box->size;
	return boxAux;
}

int VertRetangulo::calcHist(const IplImage* frame){

	float maxValue = 0;

	if(rect.x == -1 || rect.y == -1)
		if(rectSelect(frame))
			return -1;

	cvCvtColor( frame, hsv, CV_BGR2HSV );

	cvSetImageROI(hsv, rect);
	cvSetImageROI(hPlane, rect);
	cvSetImageROI(sPlane, rect);
	cvSetImageROI(vPlane, rect);

	cvCvtPixToPlane( hsv, hPlane, sPlane, vPlane, 0 );

	cvCalcHist(planes, hist);
	cvGetMinMaxHistValue(hist, 0, &maxValue, 0, 0);
	cvConvertScale(hist->bins, hist->bins, maxValue ? 255. / maxValue: 0., 0);

	cvResetImageROI(vPlane);
	cvResetImageROI(sPlane);
	cvResetImageROI(hPlane);
	cvResetImageROI(hsv);

	cvCvtPlaneToPix(hPlane, sPlane, vPlane, 0, hsv );

	return 0;
}

//N�o est� mais sendo utilizado
void VertRetangulo::updateHist(const IplImage* frame){

	float maxValue = 0;

	cvCvtColor( frame, hsv, CV_BGR2HSV );

	cvSetImageROI(hsv, rect);
	cvSetImageROI(hPlane, rect);
	cvSetImageROI(sPlane, rect);
	cvSetImageROI(vPlane, rect);

	cvCvtPixToPlane( hsv, hPlane, sPlane, vPlane, 0 );

	cvCalcHist(planes, hist);
	cvGetMinMaxHistValue(hist, 0, &maxValue, 0, 0);
	cvConvertScale(hist->bins, hist->bins, maxValue ? 255. / maxValue: 0., 0);

	cvResetImageROI(vPlane);
	cvResetImageROI(sPlane);
	cvResetImageROI(hPlane);
	cvResetImageROI(hsv);

	cvCvtPlaneToPix(hPlane, sPlane, vPlane, 0, hsv );
}

void VertRetangulo::track(IplImage* frameHSV, IplImage* frameMask){


	cvCvtPixToPlane( frameHSV, hPlane, 0, 0, 0 );
	cvCalcBackProject(planes, backProject, hist);
	cvAnd( backProject, frameMask, backProject);
	cvCamShift(backProject, rect, cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ), comp , box);
	rect = comp->rect;
}

VertRetangulo::~VertRetangulo(){

	cvReleaseImage(&hsv);	//convers�o HSV do frame capturado
	cvReleaseImage(&backProject);
	cvReleaseImage(&mask);
	cvReleaseImage(&hPlane);
	cvReleaseImage(&sPlane);
	cvReleaseImage(&vPlane);
	delete comp;
	delete box;
	cvReleaseHist(&hist);
}

int VertRetangulo::rectSelect(const IplImage* frame){

	CvRect* rectPointer = new  CvRect;
	cvNamedWindow("rectSelection");

	cout<<"Selecione na Janela 'rectSelection', Com o Bot�o Direito do Mouse o Retangulo que Contem a Cor Sobre a Qual se Deseja Realizar o Tracking/n";
	cout<<"Apos Selecionar o Retangulo Aperte Qualquer Tecla\n";

do{

	cvSetMouseCallback("rectSelection",mouseCallBack,rectPointer);
	cvShowImage("rectSelection",frame);
	
	if(rectPointer->x > 0 && rectPointer->y > 0 && rectPointer->width > 0 && rectPointer->height > 0)
		rect = cvRect(rectPointer->x, rectPointer->y, rectPointer->width, rectPointer->height);

}while(cvWaitKey(1) == -1);


	if(rect.width == 0 || rect.height == 0)
		return -1;
	else
		return 0;

	delete rectPointer;

	cvDestroyWindow("rectSelection");

}

void mouseCallBack(int event, int x, int y, int flags, void* param){

	CvRect* rectPointer = (CvRect*) param;

	if(event == CV_EVENT_RBUTTONDOWN){
		rectPointer->x = x;
		rectPointer->y = y;
		cout<<"\nRetangulo\n";
		cout<<"\n"<<rectPointer->x;
		cout<<"\n"<<rectPointer->y;
	}
	else if(event == CV_EVENT_RBUTTONUP){
		rectPointer->width = x - rectPointer->x;
		rectPointer->height = y - rectPointer->y;
		cout<<"\n"<<rectPointer->x + rectPointer->width;
		cout<<"\n"<<rectPointer->y + rectPointer->height<<"\n";
	}
}
