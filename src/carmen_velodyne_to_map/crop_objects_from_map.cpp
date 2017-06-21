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

	cv::Mat image;
	cv::Mat object;

	double xt, yt, xb, yb;
	char classe[10];

	_mkdir("/dados/dataset/objects");
	_mkdir("/dados/dataset/objects/images");

	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);// Create a window for display.

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
			while (!feof(labels))
			{
				fscanf(labels, "%s 0.00 0 0.00 %lf %lf %lf %lf 0.00 0.00 0.00 0.00 0.00 0.00 0\n", classe, &xt, &yt, &xb, &yb);
				//printf("Xt: %lf   Yt: %lf   Xb: %lf   Yb: %lf   Largura: %lf   Altura: %lf\n",xt,yt,xb,yb,yb-yt,xb-xt);

				cv::Rect rec(xt,yt,xb-xt,yb-yt);

				//printf("X: %d   Y: %d   Largura: %d   Altura: %d\n",rec.x,rec.y,rec.width,rec.height);

				//cv::Mat object = image(rec);

				//cv::imshow("Display window", object);                   // Show our image inside it.
				//cv::waitKey(5000);


				cv::rectangle(image, cv::Point(xt,yt), cv::Point(xb,yb), cv::Scalar(0,255,0), 2);
				cv::putText(image, classe, cv::Point(xt,yt-2), cv::FONT_HERSHEY_PLAIN, 2, cvScalar(0,255,0), 1);
			}

			fclose(labels);
		}
		else
			printf("Failed to open label %s\n", labelname);

		cv::imshow("Display window", image);                   // Show our image inside it.
		cv::waitKey(50);
	}

	closedir(dir);
	return 0;
}
