#include "semantic_segmentation.h"
#include <cstdio>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

//Pasta na qual os resultados serao salvos
const string RESULTS_PATH = "./results/";

vector<string> readPathFromFile(string tFile)
{
	vector<string> files;
	string line;

	ifstream file(tFile.c_str());

	if(file.is_open())
	{
		while(getline(file,line))
			files.push_back(line);
		file.close();
	}

	return files;
}

Mat cropImage(Mat img, unsigned y)
{
	Rect roi(0,0,img.cols,y);
	Mat image_roi = img(roi);
	Mat imageCropped;

	image_roi.copyTo(imageCropped);
	return imageCropped;
}

int
main(int argc, char** argv)
{
	if(argc != 2)
	{
		printf("Modo de Uso: %s <files.txt>\n Em que files.txt se refere ao arquivo que contem o path das imagens que devem ser convertidas.\n", argv[0]);
		return 1;
	}

	vector<int> compressionParams;
	compressionParams.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compressionParams.push_back(0);

	SegNet seg_net("segnet_model_driving_webdemo.prototxt", "segnet_weights_driving_webdemo.caffemodel", "camvid12.png");
	vector<string> images = readPathFromFile(argv[1]);

	if(images.size() == 0)
	{
		printf("Nao foi possivel ler o arquivo %s\n", argv[0]);
		return 2;
	}

	printf("Running...\n");

	for(unsigned i = 0; i < images.size(); i++)
	{
		printf("Predicting for: %s\n", images[i].c_str());

		Mat img = imread(images[i]);

		//Crop para cortar a regiao do carro na imagem
		img = cropImage(img, 380);

		Mat prediction = seg_net.Predict(img);
		Mat colors = seg_net.ClassesToColors(prediction);

		try 
		{
			imwrite(RESULTS_PATH + "segimg-" + to_string(i) + ".png", colors, compressionParams);
		}
		catch (runtime_error& ex) 
		{
			fprintf(stderr, "Exception converting image %i to PNG format: %s\n", i, ex.what());
			return 1;
		}
	}

	printf("Done...\n");

	return 0;
}
