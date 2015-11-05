#include "Util/log.H"
#include "Util/WorkThreadServer.H"
#include "Util/JobWithSemaphore.H"
#include "Component/ModelManager.H"
#include "Raster/GenericFrame.H"
#include "Image/Layout.H"
#include "Image/MatrixOps.H"
#include "Image/DrawOps.H"
#include "GUI/DebugWin.H"
#include "Media/FrameSeries.H"
#include "Transport/FrameInfo.H"
#include "Util/CpuTimer.H"
#include "Image/JPEGUtil.H"

#include <stdlib.h>
#include <math.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fcntl.h>

bool useTcp = true;

int main(int argc, char *argv[])
{

  ModelManager manager("Test view iMgae");

  nub::ref<OutputFrameSeries> ofs(new OutputFrameSeries(manager));
  manager.addSubComponent(ofs);

  // Parse command-line:
  if (manager.parseCommandLine(argc, argv, "<server> <port>", 2, 2) == false) return(1);
  // let's get all our ModelComponent instances started:
  manager.start();

  const char* hostname = manager.getExtraArg(0).c_str();
  int portno = atoi(manager.getExtraArg(1).c_str());

  int sockfd = -1;
  if (useTcp)
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
  else
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

  if (sockfd < 0) 
    LFATAL("ERROR opening socket");

  struct hostent* server = gethostbyname(hostname);
  if (server == NULL)
    LFATAL("ERROR, no such host as %s\n", hostname);

  /* build the server's Internet address */
  struct sockaddr_in serveraddr;
  bzero((char *) &serveraddr, sizeof(serveraddr));
  serveraddr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
      (char *)&serveraddr.sin_addr.s_addr, server->h_length);
  serveraddr.sin_port = htons(portno);

  if (useTcp)
  {
    /* connect: create a connection with the server */
    if (connect(sockfd, (struct sockaddr*)&serveraddr, sizeof(serveraddr)) < 0) 
      LFATAL("ERROR connecting");
  }

  CpuTimer timer;

  /* send the message line to the server */
  const char *startFrame = "S\n";
  int n = -1;
  if (useTcp)
    n = write(sockfd, startFrame, strlen(startFrame));
  else
    n = sendto(sockfd, startFrame, sizeof(startFrame), 0, (const struct sockaddr *)&serveraddr,sizeof(serveraddr));

  if (n < 0) 
    LFATAL("ERROR writing to socket");
  
  while(1)
  {
    timer.reset();
    for(int t=0; t<30; t++)
    {
      int n=-1;

      /* print the server's reply */
      unsigned int frameSize = 0; 
      socklen_t sin_size = sizeof(serveraddr);
      if (useTcp)
        n = recv(sockfd, &frameSize, sizeof(frameSize), MSG_WAITALL);
      else
        n = recvfrom(sockfd, &frameSize, sizeof(frameSize), MSG_WAITALL, (struct sockaddr *)&serveraddr,&sin_size);

      char buf[frameSize];
      if (useTcp)
        n = recv(sockfd, buf, frameSize, MSG_WAITALL);
      else
        n = recvfrom(sockfd, buf, frameSize, MSG_WAITALL, (struct sockaddr *)&serveraddr,&sin_size);
      LINFO("Size %i", n);


      if (n < 0) 
        LFATAL("ERROR reading from socket");

      if (n > 5000)
      {
        JPEGDecompressor jpdec;
        std::vector<unsigned char> data(n);
        for(int ii=0; ii<n; ii++)
          data[ii] = buf[ii];

        Image<PixRGB<byte> > img(320,240,ZEROS); // = jpdec.DecompressImage(data); //(320,240,ZEROS);
        img.attach((const_cast<PixRGB<byte>*> (reinterpret_cast<const PixRGB<byte>* >
                (&buf))), 320, 240);
        Image<PixRGB<byte> > tmp = img.deepcopy(); //deepcopy?
        ofs->writeRGB(tmp, "gpsOutput", FrameInfo("gpsOutput", SRC_POS));
        usleep(10000);
      }
    }
    timer.mark();
    LINFO("Total time %0.2f sec", 30/timer.real_secs());
    

  }

  close(sockfd);

  manager.stop();

}


