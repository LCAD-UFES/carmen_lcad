#include "pi_camera_driver.h"
#include <time.h>

using namespace cv;
using namespace std;


int
main(int argc, char* argv[])
{
	raspicam::RaspiCam RpiCamera;
	unsigned char *rpi_cam_data = NULL;
	int cont = 0, width = 640, height = 480;
	char frame_name[64];

	RpiCamera.setBrightness(55);
	RpiCamera.setContrast(10);
	RpiCamera.setFormat(raspicam::RASPICAM_FORMAT_RGB);
	RpiCamera.setMetering(raspicam::RASPICAM_METERING_MATRIX);
	RpiCamera.setWidth(width);
	RpiCamera.setHeight(height);
	RpiCamera.setFrameRate(30);
	RpiCamera.setHorizontalFlip(true);
	RpiCamera.setVerticalFlip(true);

	if (!RpiCamera.open())
	{
		cerr << "Error opening the camera" << endl;
		return -1;
	}
	sleep(3);

	rpi_cam_data = (unsigned char*) calloc (width * height * 3, sizeof(unsigned char));

    while (1)
    {
    	clock_t tic = clock();

		RpiCamera.grab();
		RpiCamera.retrieve (rpi_cam_data, raspicam::RASPICAM_FORMAT_RGB);

		Mat frame = Mat(height, width, CV_8UC3, rpi_cam_data);

		//imshow("Pi Camera Test", frame);
		//waitKey(1);

		sprintf(frame_name, "%s%d", "Frame_", cont);
		imwrite(frame_name, frame);

		printf("FPS: %lf\n", 1 / ((double)(clock() - tic) / CLOCKS_PER_SEC));
	}

   return 0;
}
