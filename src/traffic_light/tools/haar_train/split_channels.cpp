#include <cstdio>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/algorithm/string/trim.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

int
main(int argc, char **argv)
{

	if(argc < 2){
		std::cout << "Error! usage: " << argv[0] << " file_with_name_of_images.txt" << std::endl;
		return -1;
	}

    std::ifstream file_name_images;

    file_name_images.open(argv[1]);//arquivo com o nome das imagens para transformar

    if(file_name_images.is_open()){

    	std::string image_name;
    	cv::Mat frame_bgr;
//    	cv::Mat frame_hsv;
//    	cv::Mat frame_ycbcr;
//    	cv::Mat channel_bgr[3], channel_hsv[3], channel_ycrcb[3];
    	do{

            getline(file_name_images, image_name);
        	boost::trim(image_name);

        	frame_bgr = imread(image_name.c_str());
//        	cvtColor(frame_bgr, frame_hsv, CV_BGR2HSV);
//        	cvtColor(frame_bgr, frame_ycbcr, CV_BGR2YCrCb);

//        	cv::split(frame_bgr, channel_bgr);
//        	cv::split(frame_hsv, channel_hsv);
//        	cv::split(frame_ycbcr, channel_ycrcb);

        cv::Mat img_gr = Mat::zeros(frame_bgr.rows, frame_bgr.cols, CV_8UC1);

		   for (int y = 0; y < frame_bgr.rows; y++) {
			  for (int x = 0; x < frame_bgr.cols; x++) {

				  unsigned char blue = 0;//frame_bgr.data[3*frame_bgr.cols*y + 3*x]*0.114;
				  unsigned char green = frame_bgr.data[3*frame_bgr.cols*y + 3*x + 1]* (0.587 + 0.57);
				  unsigned char red = frame_bgr.data[3*frame_bgr.cols*y + 3*x+2]*(0.299 + 0.57);

				 img_gr.data[img_gr.cols*y+x] = red + green + blue;
			  }
		   }
        	//just to show images
//        	cv::imshow("B", channel_bgr[0]);
//        	cv::imshow("G", channel_bgr[1]);
//        	cv::imshow("R", channel_bgr[2]);
//
//           	cv::imshow("H", channel_hsv[0]);
//			cv::imshow("S", channel_hsv[1]);
//			cv::imshow("V", channel_hsv[2]);
//
//			cv::imshow("Y", channel_ycrcb[0]);
//			cv::imshow("CR", channel_ycrcb[1]);
//			cv::imshow("CB", channel_ycrcb[2]);
//
//        	cv::waitKey(0);//Wait for a keystroke in the window

        	char *pch;
        	char *name = (char*)image_name.c_str();
        	pch = strtok(name,"/");
			pch = strtok (NULL, "/");
			pch = strtok (NULL, "/");
			printf ("%s\n",pch);
			string only_image_name = pch;


//			cv::imwrite("../database_hsv/" + only_image_name, frame_hsv);
//			cv::imwrite("../database_y3/" + only_image_name, frame_y3);

        	cv::imwrite("../database_gr/" + only_image_name, img_gr);
//        	cv::imwrite("../database_g/" + only_image_name, channel_bgr[1]);
//        	cv::imwrite("../database_r/" + only_image_name, channel_bgr[2]);
//
//        	cv::imwrite("../database_h/" + only_image_name, channel_hsv[0]);
//        	cv::imwrite("../database_s/" + only_image_name, channel_hsv[1]);
//        	cv::imwrite("../database_v/" + only_image_name, channel_hsv[2]);
//
////        	cv::imwrite("../database_y/" + only_image_name, channel_ycrcb[0]);
//        	cv::imwrite("../database_cr/" + only_image_name, channel_ycrcb[1]);
//        	cv::imwrite("../database_cb/" + only_image_name, channel_ycrcb[2]);

			frame_bgr.release();
			img_gr.release();
//			frame_ycbcr.release();
//        	frame_hsv.release();
//        	channel_bgr[0].release();
//        	channel_bgr[1].release();
//        	channel_bgr[2].release();
//        	channel_hsv[0].release();
//        	channel_hsv[1].release();
//        	channel_hsv[2].release();
//        	channel_ycrcb[0].release();
//        	channel_ycrcb[1].release();
//        	channel_ycrcb[2].release();

    	}while(!file_name_images.eof());
    }

	return 0;
}
