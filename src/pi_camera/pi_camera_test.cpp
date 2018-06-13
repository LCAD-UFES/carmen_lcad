#include "pi_camera_driver.h"
#include <time.h>

using namespace cv;
using namespace std;


Mat UndistortImage(Mat input_frame, CameraParameters cam_pam)
{
	Mat output_frame;

	Mat K = (Mat_<double>(3,3) <<
			cam_pam.fx,  	 0,  	cam_pam.cx,
			0,  	cam_pam.fy, 	cam_pam.cy,
			0, 			 0,    		1);


	Mat I = (Mat_<double>(1,5) <<
			cam_pam.k1, cam_pam.k2, cam_pam.k3, cam_pam.p1, cam_pam.p2);

	undistort(input_frame, output_frame, K, I);

	return output_frame;
}

int main(int argc, char* argv[])
{
	raspicam::RaspiCam RpiCamera;
	unsigned char *rpi_cam_data = NULL;
	time_t raw_time, raw_time_2;

	CameraParameters cam_pam;
	cam_pam.width = 2592;
	cam_pam.height = 1944;
	cam_pam.fx = (1167.265655 / 2592.0) * cam_pam.width;
	cam_pam.fy = (1166.396687 / 1944.0) * cam_pam.height;
	cam_pam.cx = (1263.343460 / 2592.0) * cam_pam.width;
	cam_pam.cy = (957.987274 / 1944.0) * cam_pam.height;
	cam_pam.k1 = -0.222827;
	cam_pam.k2 = 0.055068;
	cam_pam.k3 = -0.006594;
	cam_pam.p1 = -0.000730;
	cam_pam.p2 = -0.001853;

	RpiCamera.setBrightness(55);
	RpiCamera.setContrast(10);
	RpiCamera.setFormat(raspicam::RASPICAM_FORMAT_RGB);
	RpiCamera.setMetering(raspicam::RASPICAM_METERING_MATRIX);
	RpiCamera.setWidth(1920);
	RpiCamera.setHeight(1080);
	RpiCamera.setFrameRate(30);
	RpiCamera.setHorizontalFlip(true);
	RpiCamera.setVerticalFlip(true);

	if (!RpiCamera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
	sleep(3);

	rpi_cam_data = (unsigned char* )calloc (640 * 480 * 3, sizeof(unsigned char));

    while (1)
    {
    	clock_t tic = clock();
		//capture frame
		RpiCamera.grab();
		RpiCamera.retrieve (rpi_cam_data, raspicam::RASPICAM_FORMAT_RGB);
		clock_t toc = clock();

		printf("FPS: %lf\n", 1 / (double)(toc - tic));

		imshow("frame", Mat(480, 640, CV_8UC3, rpi_cam_data, 3 * 640));
		waitKey(1);
	}

   return 0;
}
