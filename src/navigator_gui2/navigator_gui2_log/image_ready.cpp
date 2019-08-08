#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "opencv2/opencv.hpp"
#include <cstring>
#include <string>
#include <algorithm>
#include <sys/stat.h>

#include <gtk/gtk.h>
#include <gtk/gtkgl.h>

int 
main(int argc, char *argv[])
{
	if(argc<2){
		printf("Precisa-se inserir o endereço da pasta referente ao log do navigator_gui:: EX: home/lcad/carmen_lcad/data/navigator_gui2_log/2019-08-08_.../");
		return 1;
	}

	char temp[1024];
	char buffer[255];
	char folder_dir[255];
	char folder_name[255];
	char *split;
	char *cstr;
	int count, width, height;
	int text_height;
	int dist = 16;
	float font_size = 0.5;

	memset(buffer,'\0',255*sizeof(char));

	if(argv[1][strlen(argv[1])-1]!='/')
		snprintf(folder_dir,sizeof(folder_dir),"%s/", argv[1]);
	else
		snprintf(folder_dir,sizeof(folder_dir),"%s", argv[1]);

	snprintf(buffer,sizeof(buffer),"%slog_file.txt", folder_dir);

	//Split
	std::string folder_dir_str(folder_dir);
	folder_dir_str[strlen(folder_dir)-1] = '\0';
	std::string video_log_name(folder_dir_str.substr(folder_dir_str.rfind("/") + 1));

	std::ifstream file(buffer);

	if(!file.is_open())
	{
		printf("Log File could not be opened!");
		return 1;
	}
	std::string line;

	cv::Mat image;
	cv::Mat offset;
	cv::Mat concat;

	memset(temp,'\0',1024*sizeof(char));

	snprintf(temp,sizeof(temp),"%s/data/navigator_gui2_log/pictures_ready",getenv("CARMEN_HOME"));
	struct stat sb;
	if(!(stat(temp, &sb) == 0))
		mkdir(temp,0777);

	snprintf(temp,sizeof(temp),"rm %s/data/navigator_gui2_log/pictures_ready/*",getenv("CARMEN_HOME"));
	system(temp);

	while(std::getline(file,line))
	{
		try
		{
			cstr = new char [line.length()+1];
			std::replace(line.begin(),line.end(),'\t',' ');
			std::strcpy(cstr, line.c_str());

			split = std::strtok(cstr,"#");
			count = std::atoi(split);
			snprintf(temp,sizeof(temp),"%spictures/%d.jpg", folder_dir, count);

			image = cv::imread(temp, CV_LOAD_IMAGE_COLOR);

			height = image.size().height;
			width = image.size().width;
			offset = cv::Mat(height, 550, CV_8UC3, cv::Scalar(255, 255, 255));
			cv::hconcat(image, offset, concat);
			text_height = (height/2 - 100);
			split = std::strtok(NULL,"#");
			cv::line( concat, cv::Point( width, text_height-20 ), cv::Point( width+550, text_height-20), cv::Scalar( 0, 0, 0 ), 3, 8 );
			while(split!=NULL){
				cv::putText(concat, split, cv::Point(width+10,text_height), cv::FONT_HERSHEY_SIMPLEX, font_size, cv::Scalar(0,0,0), 1);
				text_height+=dist;
				split = std::strtok(NULL,"#");
			}

			cv::line( concat, cv::Point( width, text_height+10 - dist ), cv::Point( width+550, text_height+10 - dist), cv::Scalar( 0, 0, 0 ), 3, 8 );
			snprintf(temp,sizeof(temp),"%s/data/navigator_gui2_log/pictures_ready/%09d.png",getenv("CARMEN_HOME"),count);
			cv::imwrite(temp,concat);
			if(count%31==0)
				printf("%d imagens processadas.\n",count);
		}
		catch(cv::Exception& e)
		{
			printf("Erro: %d :: %s\n",count, e.msg.c_str());
		}
	}
	file.close();
//	snprintf(temp,sizeof(temp),"rm %s/data/navigator_gui2_log/video_log_navigator.mp4",getenv("CARMEN_HOME"));
//	system(temp);
	snprintf(temp,sizeof(temp),"ffmpeg -framerate 25 -pattern_type glob -i %s/data/navigator_gui2_log/pictures_ready/'*.png' %svideo_log_%s.mp4",getenv("CARMEN_HOME"),folder_dir, video_log_name.c_str());
	system(temp);

	return 0;
}
