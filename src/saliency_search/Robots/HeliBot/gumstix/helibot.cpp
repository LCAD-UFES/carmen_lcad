
#include <sys/time.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "Capture.H"

bool useTcp = true;

int initServer()
{
  int sock;
  int istrue=1;  

  struct sockaddr_in server_addr;
   
  if (useTcp)
  {
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      perror("Socket");
      exit(1);
    }
  } else {
    //UDP
    if ((sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
      perror("Socket");
      exit(1);
    }
  }


  if (setsockopt(sock,SOL_SOCKET,SO_REUSEADDR,&istrue,sizeof(int)) == -1) {
    perror("Setsockopt");
    exit(1);
  }
        
  server_addr.sin_family = AF_INET;         
  server_addr.sin_port = htons(5000);     
  server_addr.sin_addr.s_addr = INADDR_ANY; 
  bzero(&(server_addr.sin_zero),8); 

  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1) {
    perror("Unable to bind");
    exit(1);
  }

  if (useTcp) //No need to listen any more
  {
    if (listen(sock, 5) == -1) {
      perror("Listen");
      exit(1);
    }

    printf("\nTCPServer Waiting for client on port 5000\n");
    fflush(stdout);
  }


  return sock;

}


int main(int argc, char** argv)
{

  Capture camera(320,240,15);

  camera.initCapture();

  int sock = initServer();

  while(1)
  {  
    char send_data [1024] , recv_data[1024];       

    int connected = -1;
    struct sockaddr_in client_addr;    
    socklen_t sin_size = sizeof(struct sockaddr_in);

    if (useTcp)
      connected = accept(sock, (struct sockaddr *)&client_addr,&sin_size);

    printf("\n I got a connection from (%s , %d)",
        inet_ntoa(client_addr.sin_addr),ntohs(client_addr.sin_port));

    int width = 320;
    int height = 240;
    unsigned char img[width*height*3];

    int bytes_recieved = -1;
    if (useTcp)
      bytes_recieved = recv(connected,recv_data,1024,0);
    else
      bytes_recieved = recvfrom(sock, recv_data, 1024, 0, (struct sockaddr *)&client_addr,&sin_size);

    while (1)
    {
      struct timeval real1, real2;
         
      gettimeofday(&real1, /* timezone */ 0);
      for(uint i=0; i<30; i++)
      {
        
        unsigned int frameSize = 0;
        unsigned char* img = camera.grabFrameRaw(frameSize);

        if (useTcp)
        {
          send(connected, &frameSize, sizeof(frameSize), 0);  
          send(connected, img, frameSize, 0);  
        } else {
          sendto(sock, &frameSize, sizeof(frameSize), 0, (const struct sockaddr*)&client_addr,sin_size);
          sendto(sock, img ,frameSize, 0, (const struct sockaddr*)&client_addr,sin_size);
        }
      }
      gettimeofday(&real2, /* timezone */ 0);
      const double real_secs =
        (real2.tv_sec - real1.tv_sec)
        + (real2.tv_usec - real1.tv_usec)
        / 1000000.0;

      char msg[255];
      printf("fps %0.2f\n", 30.0/real_secs);
    }
    close(connected);
  }       

  close(sock);


  return 0;
}



