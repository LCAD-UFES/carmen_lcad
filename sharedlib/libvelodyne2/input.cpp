/** \file
 *
 *  Input classes for the Velodyne HDL-64E 3D LIDAR:
 * 
 *     Input -- virtual base class than can be used to access the data
 *              independently of its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>

#include "input.h"

namespace velodyne_driver
{
  static const size_t scan_packet_size = VELODYNE_MSG_BUFFER_SIZE;
  static const size_t gps_packet_size = VELODYNE_GPS_BUFFER_SIZE;
  ////////////////////////////////////////////////////////////////////////
  // InputSocket class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh private node handle for driver
   *  @param udp_port UDP port number to connect
   */
  InputSocket::InputSocket(unsigned short udp_port):
    Input()
  {
    sockfd_ = -1;
    port_number = udp_port;

    // connect to Velodyne UDP port
	printf("opening udp socket: port %d\n", udp_port);
    sockfd_ = socket(AF_INET, SOCK_DGRAM,  0);
    if (sockfd_ == -1)
    {
      printf("error: socket");
      return;
    }
  
    sockaddr_in my_addr;                     // my address information
    memset(&my_addr, 0, sizeof(my_addr));    // initialize to zeros
    my_addr.sin_family = AF_INET;            // host byte order
    my_addr.sin_port = htons(udp_port);      // short, in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;    // automatically fill in my IP


    if (bind(sockfd_, (sockaddr *)&my_addr, sizeof(sockaddr)))
    {
      printf("error: bind");
      return;
    }
  
    /*DEBUG: printf("Velodyne socket fd is %d\n", sockfd_);*/
  }

  /** @brief destructor */
  InputSocket::~InputSocket(void)
  {
    (void) close(sockfd_);
  }

  /** @brief Get one velodyne packet. */
  int InputSocket::getScanPacket(velodyne_packet_t *pkt, int velodyne_udp_port)
  {
	if (port_number == velodyne_udp_port)
	{
		struct pollfd fds[1];
		fds[0].fd = sockfd_;
		fds[0].events = POLLIN;
		static const int POLL_TIMEOUT = 3000; // one second (in msec)

		while (true)
		{
			do
			{
			  int retval = poll(fds, 1, POLL_TIMEOUT);
			  if (retval < 0)             // poll() error?
			  {
				if (errno != EINTR)
				  printf("poll() error: %s\n", strerror(errno));
				return 1;
			  }

			  if (retval == 0)            // poll() timeout?
			  {
				printf("velodyne scan poll() timeout\n");
				return 1;
			  }

			  if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
			  {
				printf("poll() reports velodyne error\n");
				return 1;
			  }

			 } while ((fds[0].revents & POLLIN) == 0);

			 // Receive packets that should now be available from the
			 // socket using a blocking read.
			 ssize_t nbytes = recvfrom(sockfd_, pkt, scan_packet_size,  0, NULL, NULL);

			 if ((size_t) nbytes == scan_packet_size)
			 {
			   // read successful, done now
			   break;
			 }

			 //DEBUG: printf("incomplete Velodyne packet read: %d bytes\n", nbytes);
		}

		return 0;
	}
	else
		return -1;
  }

  /** @brief Get one velodyne packet. */
  int InputSocket::getGpsPacket(velodyne_gps_packet_t *pkt, int velodyne_gps_udp_port)
  {
	  if(port_number == velodyne_gps_udp_port)
	  {
		  struct pollfd fds[1];
		  fds[0].fd = sockfd_;
		  fds[0].events = POLLIN;
		  static const int POLL_TIMEOUT = 3000; // one second (in msec)

		  while (true)
		  {
			  do
			  {
				int retval = poll(fds, 1, POLL_TIMEOUT);
				if (retval < 0)             // poll() error?
				{
				  if (errno != EINTR)
					printf("poll() error: %s\n", strerror(errno));
				  return 1;
				}

				if (retval == 0)            // poll() timeout?
				{
				  printf("velodyne gps poll() timeout\n");
				  return 1;
				}

				if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
				{
				  printf("poll() reports velodyne error\n");
				  return 1;
				}

			   } while ((fds[0].revents & POLLIN) == 0);

			   // Receive packets that should now be available from the
			   // socket using a blocking read.
			   ssize_t nbytes = recvfrom(sockfd_, pkt, gps_packet_size,  0, NULL, NULL);

			   if ((size_t) nbytes == gps_packet_size)
			   {
				 // read successful, done now
				 break;
			   }

			   //DEBUG: printf("incomplete Velodyne packet read: %d bytes\n", nbytes);
			}

		  return 0;
	  }
	  else
		  return -1;
  }

  void InputSocket::printGpsPacket(velodyne_gps_packet_t packet) {
	  	printf("Gyro    -> 1: %x, 2: %x, 3: %x\n", packet.gyro1, packet.gyro2, packet.gyro3);
	  	printf("Temp    -> 1: %x, 2: %x, 3: %x\n", packet.temp1, packet.temp2, packet.temp3);
	  	printf("Accel_X -> 1: %x, 2: %x, 3: %x\n", packet.accel1_x, packet.accel2_x, packet.accel3_x);
	  	printf("Accel_Y -> 1: %x, 2: %x, 3: %x\n", packet.accel1_y, packet.accel2_y, packet.accel3_y);

	  	printf("NMEA:");
	  	for(int i=0; i < 72; i++)
	  		printf("%c", packet.nmea_sentence[i]);
	  	printf("\n");

	  	printf("Timestamp:");
	  	for(int i=0; i < 4; i++)
			printf("%d ", packet.gps_timestamp[i]);
	  	printf("\n");
  }

} // velodyne namespace
