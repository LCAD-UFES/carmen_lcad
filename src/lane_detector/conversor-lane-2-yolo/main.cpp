#include "tinyxml.h"
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;
void lerxml(string file, string base)
{
	string pFilename = base + "/" + file + "/groundtruth.xml";
	string arquivo = base + "/" + file + "/config.xml";
  	TiXmlDocument doc( pFilename.c_str() );
	TiXmlDocument doc2( arquivo.c_str() );
	bool loadOkay = doc.LoadFile();
  	if (!loadOkay)
 	{
		cout << "Não foi possível abrir o arquivo de groundtruth xml\n";
		return;
	}
	loadOkay = doc2.LoadFile();
	if (!loadOkay)
 	{
		cout << "Não foi possível abrir o arquivo de configuracao xml\n";
		return;
	}
	TiXmlElement *currentElement2 = doc2.RootElement();
	TiXmlElement *currentElement = doc.RootElement();
	TiXmlElement *pF2 = currentElement2 -> FirstChildElement("dataset");
			cout << "ola\n";
	TiXmlElement *pROI = pF2 -> FirstChildElement("region_of_interest");
	TiXmlElement *pFrame = currentElement -> FirstChildElement("frames");
	TiXmlElement *pF = pFrame -> FirstChildElement("frame");
	TiXmlElement *pPosition = pF -> FirstChildElement("position");
	TiXmlElement *pLeft = pPosition -> FirstChildElement("left");
    TiXmlElement *pRight = pPosition -> FirstChildElement("right");
	TiXmlElement *pP1 = pLeft -> FirstChildElement("p1");
	float p1l = atof (pP1 -> GetText());
	TiXmlElement *pP4r = pRight -> FirstChildElement("p4");
	float p4r = atof (pP4r -> GetText());
	string p4rs = pP4r -> GetText();
	TiXmlElement *pP3r = pRight -> FirstChildElement("p3");
	float p3r = atof (pP3r -> GetText());
	TiXmlElement *pP2r = pRight -> FirstChildElement("p2");
	float p2r = atof (pP2r -> GetText());
	TiXmlElement *pP1r = pRight -> FirstChildElement("p1");
	float p1r = atof (pP1r -> GetText());
	TiXmlElement *pP4 = pLeft -> FirstChildElement("p4");
	float p4l = atof (pP4 -> GetText());
	TiXmlElement *pP2 = pLeft -> FirstChildElement("p2");
	float p2l = atof (pP2 -> GetText());
	TiXmlElement *pP3 = pLeft -> FirstChildElement("p3");
	float p3l = atof (pP3 -> GetText());
	float h = atof (pROI -> Attribute("height"));
	float y = atof (pROI -> Attribute("y")); 
	float centerx1l = (p1l + p2l)/ (2*640.); 
	float centery1 = (y + (h / 8)) / 480.;
	float largura1l = (p1l - p2l) / 640.;
	double altura = (h/4) / 480. ;
	double altura2 = (h/2) / 480. ;
	float centerx2l = (p2l + p3l) / (2*640.); 
	float centery2 = (y + (3*h / 8)) / 480.;
	float largura2l = (p2l - p3l) / 640.;
	float centerx3l = (p3l + p4l) / (2*640.); 
	float centery3 = (y - 1 + (3 * h / 4)) / 480.;
	float largura3l = (p3l - p4l) / 640.;
	float centerx1r = (p1r + p2r) / (2*640.);
	float largura1r =  (p2r - p1r) / 640.;
	float centerx2r = (p2r + p3r) / (2*640.);
	float largura2r =  (p3r - p2r) / 640.;
	float centerx3r = (p3r + p4r) / (2*640.);
	float largura3r =  (p4r - p3r) / 640.;
	int cnt = 0;
        char lixo[50];
	sprintf(lixo,"%d",cnt);
	string lixo1 = lixo;
	while (pF != NULL)
	{
	   if (p4rs.compare("nan") != 0)
	   {
	   		string nome = base + "/" + file + "/labels/" + "lane_" + lixo1 + ".txt"; 
       		FILE* pFILE = fopen (nome.c_str(),"w");
	   		fprintf(pFILE, "0 %f %f %f %f\n", centerx1l, centery1, largura1l, altura);
		    fprintf(pFILE, "0 %f %f %f %f\n", centerx2l, centery2, largura2l, altura);
	   		fprintf(pFILE, "0 %f %f %f %f\n", centerx3l, centery3, largura3l, altura2);
	   		fprintf(pFILE, "1 %f %f %f %f\n", centerx1r, centery1, largura1r, altura);
		    fprintf(pFILE, "1 %f %f %f %f\n", centerx2r, centery2, largura2r, altura);
	   		fprintf(pFILE, "1 %f %f %f %f", centerx3r, centery3, largura3r, altura2);
	   		fclose(pFILE);
	   }
	   pF = pF->NextSiblingElement();
	   cnt++;
       pPosition = pF -> FirstChildElement("position");
	   pLeft = pPosition -> FirstChildElement("left");
	   pRight = pPosition -> FirstChildElement("right");
	   pP1 = pLeft -> FirstChildElement("p1");
	   string p1 = pP1 -> GetText();
	   p1l = atof (pP1 -> GetText());
	   pP4r = pRight -> FirstChildElement("p4");
	   p4r = atof (pP4r -> GetText());
	   p4rs = pP4r -> GetText();
	   pP3r = pRight -> FirstChildElement("p3");
	   p3r = atof (pP3r -> GetText());
	   pP2r = pRight -> FirstChildElement("p2");
	   p2r = atof (pP2r -> GetText());
	   pP1r = pRight -> FirstChildElement("p1");
	   p1r = atof (pP1r -> GetText());
	   pP4 = pLeft -> FirstChildElement("p4");
	   p4l = atof (pP4 -> GetText());
	   pP2 = pLeft -> FirstChildElement("p2");
	   p2l = atof (pP2 -> GetText());
	   pP3 = pLeft -> FirstChildElement("p3");
	   p3l = atof (pP3 -> GetText());
	   centerx1l = (p1l + p2l) / (2*640.); 
	   largura1l = (p1l - p2l) / 640.;
       centerx2l = (p2l + p3l) / (2*640.); 
       largura2l = (p2l - p3l) / 640.;
	   centerx3l = (p3l + p4l) / (2*640.);
	   largura3l = (p3l - p4l) / 640.; 
	   centerx1r = (p1r + p2r) / (2*640.);
	   largura1r =  (p2r - p1r) / 640.;
	   centerx2r = (p2r + p3r) / (2*640.);
	   largura2r =  (p3r - p2r) / 640.;
	   centerx3r = (p3r + p4r) / (2*640.);
	   largura3r =  (p4r - p3r) / 640.;
	   	cout << centery1 << "\n";
	   sprintf(lixo,"%d",cnt);
	   lixo1=lixo;
	}
}
int main (int argc,char * argv[])
{
	//caminho da base, ex : /dados/lane_detection/Base_Rodrigo 	
	string base = argv[1];
	//nome da pasta, ex: BR_S01
	string file = argv[2];
		cout << "ola\n";
	lerxml(file, base);
	return 0;
}
