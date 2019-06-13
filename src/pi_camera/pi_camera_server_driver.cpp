#include "pi_camera_driver.h"

#define PORT 3457

int server_fd;


void
extract_camera_configuration(char *cam_config, int &image_width, int &image_height, int &frame_rate, int &brightness, int &contrast)
{
	char *token;

	token = strtok(cam_config, "*");

	printf ("--- Connected! Widith: %s ", token);
	image_width = atoi(token);

	token = strtok (NULL, "*");
	printf ("Height: %s ", token);
	image_height = atoi(token);

	token = strtok (NULL, "*");
	printf ("Frame Rate: %s ", token);
	frame_rate = atoi(token);

	token = strtok (NULL, "*");
	printf ("Brightness: %s ", token);
	brightness = atoi(token);

	token = strtok (NULL, "*");
	printf ("Contrast: %s ---\n", token);
	contrast = atoi(token);
}


void
set_camera_configurations(raspicam::RaspiCam &RpiCamera, int image_width, int image_height, int frame_rate, int brightness, int contrast)
{
    if (RpiCamera.isOpened())
            RpiCamera.release();
    
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
		cerr << "Could not open the camera!\n" << endl;
		exit(0);
	}
}


int
connect_with_client(raspicam::RaspiCam &RpiCamera, char* cam_config, int &image_width, int &image_height,
		int &image_size, int &frame_rate, int &brightness, int &contrast)
{
    int new_socket;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("--- Socket Failed ---\n");
        return (-1);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
        perror("--- Setsockopt Failed ---\n");
        return (-1);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons( PORT );
    
    // Forcefully attaching socket to the port defined
    if (bind(server_fd, (struct sockaddr*) &address, sizeof(address)) < 0)
    {
        perror("--- Bind Failed ---\n");
        return (-1);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("-- Listen Failed ---\n");
        return (-1);
    }
    
    printf("--- Waiting for connection! --\n");
    if ((new_socket = accept(server_fd, (struct sockaddr*) &address, (socklen_t*) &addrlen)) < 0)
    {
        perror("--- Accept Failed ---\n");
        return (-1);
    }
    printf("--- Connection established successfully! ---\n");

	recv(new_socket, cam_config, 64, MSG_WAITALL);

	extract_camera_configuration(cam_config, image_width, image_height, frame_rate, brightness, contrast);

    //set_camera_configurations(RpiCamera, image_width, image_height, frame_rate, brightness, contrast);

	image_size = image_width * image_height * 3;
	
    return (new_socket);
}


int
main()
{
	raspicam::RaspiCam RpiCamera;
    raspicam::RaspiCam *aux;
	char cam_config[64];
	int result = 0, image_width = 0, image_height = 0, frame_rate = 0, brightness = 0, contrast = 0, image_size = 0;

	int pi_socket = connect_with_client(RpiCamera, cam_config, image_width, image_height, image_size, frame_rate, brightness, contrast);


	unsigned char *rpi_cam_data = (unsigned char*) calloc (image_size, sizeof(unsigned char)); // TODO deve ficar apos a leitura do cam config
	
	set_camera_configurations(RpiCamera, image_width, image_height, frame_rate, brightness, contrast);

	while (1)
	{
		RpiCamera.grab();     // Capture frame
		RpiCamera.retrieve (rpi_cam_data, raspicam::RASPICAM_FORMAT_RGB);

		result = send(pi_socket, rpi_cam_data, image_size, MSG_NOSIGNAL);  // Returns number of bytes read, 0 in case of connection lost, -1 in case of error

		if (result == -1)
		{
			printf("--- Disconnected ---\n");
            close(server_fd);
            sleep(3);
            
            pi_socket = connect_with_client(RpiCamera, cam_config, image_width, image_height, image_size, frame_rate, brightness, contrast);
        }

        //imshow("Pi Cam Server", Mat(image_height, image_width, CV_8UC3, rpi_cam_data, 3 * image_width));   waitKey(1);
    }

   return (0);
}
