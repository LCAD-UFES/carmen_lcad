/*
 * crop_objects_from_map.cpp
 *
 *  Created on: June 10, 2017
 *      Author: diego
 */

#include <carmen/carmen.h>
#include <stdio.h>
#include <dirent.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

static void _mkdir(const char *dir)
{
	char tmp[256];
	char *p = NULL;
	size_t len;

	snprintf(tmp, sizeof(tmp),"%s",dir);
	len = strlen(tmp);
	if(tmp[len - 1] == '/')
		tmp[len - 1] = 0;
	for(p = tmp + 1; *p; p++)
		if(*p == '/')
		{
			*p = 0;
			mkdir(tmp, S_IRWXU);
			*p = '/';
		}
	mkdir(tmp, S_IRWXU);
}

int main()
{
	DIR *dir;
	struct dirent *lsdir;
	FILE *labels;

	double timestamp;
	char imagename[256];
	char labelname[256];
	char objectname[256];

	cv::Mat image;
	cv::Mat object;

	double xt, yt, xb, yb;
	char classe[10];
	int count;

	_mkdir("/dados/dataset/objects");
	_mkdir("/dados/dataset/objects/images");

	setlocale(LC_ALL, "C");

	dir = opendir("/dados/dataset/labels");

	/* open all the files and directories within directory*/
	while ( ( lsdir = readdir(dir) ) != NULL )
	{
		sscanf(lsdir->d_name,"%lf", &timestamp);
		sprintf(imagename,"/dados/dataset/map/%lf.png", timestamp);
		sprintf(labelname,"/dados/dataset/labels/%lf.txt", timestamp);

		image = cv::imread(imagename, CV_LOAD_IMAGE_COLOR);   // open image

		if (image.empty())
		{
			printf("Failed to open image %s\n", imagename);
			continue;
		}

		//printf("Linhas: %d   Colunas: %d\n",image.rows,image.cols);

		labels = fopen(labelname, "r");

		if (labels)
		{
			count = 1;
			while (!feof(labels))
			{
				fscanf(labels, "%s 0.00 0 0.00 %lf %lf %lf %lf 0.00 0.00 0.00 0.00 0.00 0.00 0\n", classe, &xt, &yt, &xb, &yb);
				//printf("Xt: %lf   Yt: %lf   Xb: %lf   Yb: %lf   Largura: %lf   Altura: %lf\n",xt,yt,xb,yb,yb-yt,xb-xt);

				cv::Rect roi(xt,yt,xb-xt,yb-yt);
				//printf("Timestamp: %lf\nX: %d   Y: %d   Largura: %d   Altura: %d\n",timestamp,roi.x,roi.y,roi.width,roi.height);

				if (0 <= roi.x && 0 <= roi.width && roi.x + roi.width <= image.cols && 0 <= roi.y && 0 <= roi.height && roi.y + roi.height <= image.rows)
				{
					sprintf(objectname,"/dados/dataset/objects/images/%lf.%03d-%s.png", timestamp,count,classe);
					cv::imwrite(objectname, image(roi));
					count++;

					//printf("Objeto gerado com sucesso!!!\n");
				}
				else
				{
					printf("The bounding box is out of map\n");
				}
			}

			fclose(labels);
		}
		else
		{
			printf("Failed to open label %s\n", labelname);
		}
	}

	closedir(dir);
	return 0;
}
