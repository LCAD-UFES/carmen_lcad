
#include "../visual_car_tracking/visual_car_tracking.h"



/* uses openCV with an cascade classifier to identify cars at an image.
 * needs to put car.xml into the folder for it to work
 * https://github.com/andrewssobral/vehicle_detection_haarcascades <- how to build your own cars.xml
 * TODO */
void cascade_car_finder(carmen_bumblebee_basic_stereoimage_message *message)
{
	/* transform messages received from the bumblebee to an format suitable to use with openCV */
	cv::Mat image(message->height, message->width, CV_8UC3, message->raw_right);
	/* Conversion because openCV uses BGR for the colors*/
	cv::cvtColor(image, image, CV_RGB2BGR);

	/* Creating new window to display the original image*/
	cv::namedWindow( "window1", 1 );   imshow( "window1", image );

	/* Load car cascade (.xml file) */
	cv::CascadeClassifier car_cascade;
	if(!car_cascade.load( "cars.xml" )){
		perror("Error loading xml");
	}

	/* Detect cars */
	std::vector<cv::Rect> cars;
	car_cascade.detectMultiScale( image, cars, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

	//std::cout<<"Number of cars detected: "<<cars.size()<<std::endl;

	/* Draw rectangles on the detected cars */
	for( unsigned int i = 0; i < cars.size(); i++ )
	{
		cv::rectangle(image,cars[i],cv::Scalar(0,0,255),2);
	}
	/* now displaying the image with the rectangles at the cars */
	imshow( "Detected car", image );

	cv::waitKey(0);


}

