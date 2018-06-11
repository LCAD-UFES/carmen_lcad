#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include "pi_camera_driver.h"

using namespace cv;
using namespace std;

#define PORT 3457
//#define DISPLAY_IMAGE


int
stablished_connection_with_client()
{
    int server_fd, new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
      
    // Forcefully attaching socket to the port defined
    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    printf("Connection stablished sucessfully!\n");	
	
    return (new_socket);
}


Mat
undistort_image(Mat input_frame, CameraParameters cam_pam)
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

raspicam::RaspiCam
set_camera_configurations()
{
	raspicam::RaspiCam RpiCamera;

	RpiCamera.setBrightness(55);
	RpiCamera.setContrast(10);
	RpiCamera.setFormat(raspicam::RASPICAM_FORMAT_RGB);
	RpiCamera.setMetering(raspicam::RASPICAM_METERING_MATRIX);
	RpiCamera.setWidth(640);
	RpiCamera.setHeight(480);
	RpiCamera.setFrameRate(30);
	RpiCamera.setHorizontalFlip(true);
	RpiCamera.setVerticalFlip(true);

	return (RpiCamera);
}

CameraParameters
set_camera_parameters()
{
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

	return (cam_pam);
}


int
main()
{
	raspicam::RaspiCam RpiCamera = set_camera_configurations();


	if (!RpiCamera.open())
	{
		cerr << "Error while opening the camera!\n" << endl;
		return -1;
	}

	unsigned char *rpi_cam_data = (unsigned char*) calloc (640 * 480 * 3, sizeof(unsigned char));
	
	int new_socket = stablished_connection_with_client();

	while (1)
	{
		RpiCamera.grab();     // Capture frame
		RpiCamera.retrieve (rpi_cam_data, raspicam::RASPICAM_FORMAT_RGB);

		send(new_socket, rpi_cam_data, 640 * 480 * 3, MSG_NOSIGNAL);

		#ifdef DISPLAY_IMAGE
		if (rpi_cam_data != NULL)
		{
			imshow("Pi Cam Server", Mat(480, 640, CV_8UC3, rpi_cam_data, 3 * 640));
			waitKey(1);
		}
   	   	#endif
	}

   return (0);
}
