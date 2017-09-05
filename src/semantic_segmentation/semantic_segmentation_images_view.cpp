#include <cstdio>
#include <fstream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

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

int main (int argc, char **argv)
{
	if(argc != 3)
	{
		printf("Uso: %s <arquivo-imagens.txt> <arquivo-imagens-segmentadas.txt>\n", argv[0]);
		printf("Em que os arquivos de texto correspondem aos arquivos que contém o caminho até as imagens correspondentes. Uma por linha\n");

		return 1;
	}

	vector<string> images;
	vector<string> segImages;

	images = readPathFromFile(argv[1]);

	if(images.size() == 0)
	{
		printf("Nao foi possível ler o arquivo de imagens\n");

		return 1;
	}

	segImages = readPathFromFile(argv[2]);

	if(segImages.size() == 0)
	{
		printf("Nao foi possível ler o arquivo de imagens segmentadas\n");

		return 1;
	}

	for(unsigned i = 0; i < images.size(); i+=2)
	{
		Mat image = imread(images[i], CV_LOAD_IMAGE_COLOR);
		Mat segImage = imread(segImages[i], CV_LOAD_IMAGE_COLOR);

		Mat imageResized(Size(image.cols,image.rows), segImage.type());
		resize(segImage,imageResized,imageResized.size());

		imshow("image", image);

		imshow("segImage", imageResized);

		if(i == 0)
		{
			waitKey(-1);
		}
		else
			waitKey(20);
	}

	return 0;
}
