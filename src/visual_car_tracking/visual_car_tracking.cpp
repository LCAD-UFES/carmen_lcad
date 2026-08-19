
#include "../visual_car_tracking/visual_car_tracking.h"
/* Migração Ubuntu 26.04: símbolos da API C do OpenCV (IplImage, cvScalar,
   CV_FONT_*, cvDestroyAllWindows, ...) continuam existindo no OpenCV 4, mas só nestes
   headers *_c.h — antes chegavam por inclusão transitiva do opencv/cv.h. */
#include <opencv2/imgproc/imgproc_c.h>



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

	/* Load car cascade (.xml file) */
	cv::CascadeClassifier car_cascade;
	if(!car_cascade.load( "cars.xml" )){
		perror("Error loading cars.xml");
	}

	/* Detect cars */
	std::vector<cv::Rect> cars;
	/* Migração Ubuntu 26.04: CV_HAAR_SCALE_IMAGE (API C do Haar, objdetect/objdetect_c.h) foi
	   removida no OpenCV 4. O equivalente da API C++ é cv::CASCADE_SCALE_IMAGE, mesmo valor. */
	car_cascade.detectMultiScale( image, cars, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30) );

	/* Draw rectangles on the detected cars */
	for( unsigned int i = 0; i < cars.size(); i++ )
	{
		cv::rectangle(image,cars[i],cv::Scalar(0,0,255),2);
	}

	/* now displaying the image with the rectangles at the cars */
	imshow( "Detected car", image );

	cv::waitKey(0);

}

