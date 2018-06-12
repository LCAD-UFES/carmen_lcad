#include "pi_camera_driver.h"

#define PORT 3457

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
        return (-1);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("setsockopt");
        return (-1);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
      
    // Forcefully attaching socket to the port defined
    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        perror("bind failed");
        return (-1);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        return (-1);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        perror("accept");
        return (-1);
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


void
extract_camera_configuration(char *cam_config, int &image_width, int &image_height, int &frame_rate, int &brightness, int &contrast)
{
	char *token;

	token = strtok(cam_config, "*");

	printf ("Widith %s\n", token);
	image_width = atoi(token);

	token = strtok (NULL, "*");
	printf ("Height %s\n", token);
	image_height = atoi(token);

	token = strtok (NULL, "*");
	printf ("Frame Rate %s\n", token);
	frame_rate = atoi(token);

	token = strtok (NULL, "*");
	printf ("Brightness %s\n", token);
	brightness = atoi(token);

	token = strtok (NULL, "*");
	printf ("Contrast %s\n", token);
	contrast = atoi(token);
}


raspicam::RaspiCam
set_camera_configurations(int image_width, int image_height, int frame_rate, int brightness, int contrast)
{
	raspicam::RaspiCam RpiCamera;
	
	RpiCamera.setWidth(image_width);
	RpiCamera.setHeight(image_height);
	RpiCamera.setFrameRate(frame_rate);
	RpiCamera.setBrightness(brightness);
	RpiCamera.setContrast(contrast);
	RpiCamera.setFormat(raspicam::RASPICAM_FORMAT_RGB);
	RpiCamera.setMetering(raspicam::RASPICAM_METERING_MATRIX);
	RpiCamera.setHorizontalFlip(true);
	RpiCamera.setVerticalFlip(true);

	if (!RpiCamera.open())
	{
		cerr << "Error while opening the camera!\n" << endl;
		exit(0);
	}

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
	char cam_config[64];
	int image_width = 0, image_height = 0, frame_rate = 0, brightness = 0, contrast = 0, image_size = 0;

	int pi_socket = stablished_connection_with_client();

	recv(pi_socket, cam_config, 64, MSG_WAITALL);

	extract_camera_configuration(cam_config, image_width, image_height, frame_rate, brightness, contrast);
	
	raspicam::RaspiCam RpiCamera = set_camera_configurations(image_width, image_height, frame_rate, brightness, contrast);
	image_size = image_width * image_height * 3;

	unsigned char *rpi_cam_data = (unsigned char*) calloc (image_size, sizeof(unsigned char));
	
	while (1)
	{
		RpiCamera.grab();     // Capture frame
		RpiCamera.retrieve (rpi_cam_data, raspicam::RASPICAM_FORMAT_RGB);

		int send_result = send(pi_socket, rpi_cam_data, image_size, MSG_NOSIGNAL);
		if (send_result == -1)
		{
			// Possibly disconnected. Trying to reconnect...
			pi_socket = stablished_connection_with_client();
			while (pi_socket == -1)
			{
				printf("Lost connection... Trying to reconnect\n");
				sleep(1);
				pi_socket = stablished_connection_with_client();
			}
		}

		//imshow("Pi Cam Server", Mat(image_height, image_width, CV_8UC3, rpi_cam_data, 3 * image_width));
		//waitKey(1);
	}

   return (0);
}
