/*******************GENERATE_GT***********************

 Compile:
 g++ -std=c++0x -o pi_camera_client_driver pi_camera_client_driver.cpp -W -Wall `pkg-config --cflags opencv` -O4 `pkg-config --libs opencv`

 *************************************************/

#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/legacy/legacy.hpp>

#define PORT "3457"
#define TCP_IP_ADDRESS			"192.168.0.15"


using namespace std;
using namespace cv;


int 
main()
{
    struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
    struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
    int sock = 0, valread, status;
    unsigned char raw_image[640 * 480 * 3] = {0};
    
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
  
    memset(&host_info, 0, sizeof host_info);
      
    host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
    host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

    status = getaddrinfo(TCP_IP_ADDRESS, PORT, &host_info, &host_info_list);
    
    if (status != 0)
    {
        printf("Erro no Get_Addrinfo\n");
        return (-1);
    }

    status = connect(sock, host_info_list->ai_addr, host_info_list->ai_addrlen);
    
    if(status < 0)
    {
        printf("\nConnection Failed \n");
        return (-1);
    }
    
    
    printf("Connection stablished!\n");
    
    //Mat open_cv_image = Mat(Size(640, 480), CV_8UC3);              // CV_32FC3 float 32 bit 3 channels (to char image use CV_8UC3)
    Mat open_cv_image;
    
    while (1)
    {
	valread = recv(sock, raw_image, 640 * 480 * 3, MSG_WAITALL);
	
	//raw_image[640*480*3-1] = '\0';
	
	//printf("dddd\n\n\n");
	
	//open_cv_image = Mat(480, 640, CV_8UC3, raw_image, 3 * 640);
	//imshow("Neural Object Detector", open_cv_image);
	//waitKey(1);
    }    
    
    return 0;
}


