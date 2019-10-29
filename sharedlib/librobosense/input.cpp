/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */
#include "input.h"

extern volatile sig_atomic_t flag;
namespace rslidar_driver
{
static const size_t packet_size = RSLIDAR_MSG_BUFFER_SIZE;

////////////////////////////////////////////////////////////////////////
// Input base class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number.
 */
Input::Input(rslidar_param private_nh, uint16_t port) : private_nh_(private_nh), port_(port)
{
	npkt_update_flag_ = false;
	cur_rpm_ = 600;
	return_mode_ = 1;

	devip_str_ = private_nh.device_ip;
	if (!devip_str_.empty())
	{
		printf("[driver][input] accepting packets from IP address: %s\n", devip_str_);
	}
}

int Input::getRpm(void)
{
	return cur_rpm_;
}

int Input::getReturnMode(void)
{
	return return_mode_;
}

bool Input::getUpdateFlag(void)
{
	return npkt_update_flag_;
}

void Input::clearUpdateFlag(void)
{
	npkt_update_flag_ = false;
}
////////////////////////////////////////////////////////////////////////
// InputSocket class implementation
////////////////////////////////////////////////////////////////////////

/** @brief constructor
 *
 *  @param private_nh ROS private handle for calling node.
 *  @param port UDP port number
 */
InputSocket::InputSocket(rslidar_param private_nh, uint16_t port) : Input(private_nh, port)
{
	sockfd_ = -1;

	if (!devip_str_.empty())
	{
		inet_aton(devip_str_.c_str(), &devip_);
	}

	printf("[driver][socket] Opening UDP socket: port %c\n", port);
	sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
	if (sockfd_ == -1)
	{
		printf("[driver][socket] create socket fail");
		return;
	}

	int opt = 1;
	if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
	{
		printf("[driver][socket] setsockopt fail");
		return;
	}

	sockaddr_in my_addr;                   // my address information
	memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
	my_addr.sin_family = AF_INET;          // host byte order
	my_addr.sin_port = htons(port);        // port in network byte order
	my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

	if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
	{
		printf("[driver][socket] socket bind fail");
		return;
	}

	if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	{
		printf("[driver][socket] fcntl fail");
		return;
	}
}

/** @brief destructor */
InputSocket::~InputSocket(void)
{
	(void)close(sockfd_);
}

/** @brief Get one rslidar packet. */
int InputSocket::getPacket(rslidarPacket_t* pkt, const double time_offset)
{
	double time1 = carmen_get_time();
	struct pollfd fds[1];
	fds[0].fd = sockfd_;
	fds[0].events = POLLIN;
	static const int POLL_TIMEOUT = 1000;  // one second (in msec)

	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);
	while (flag == 1)
	{
		// Receive packets that should now be available from the
		// socket using a blocking read.
		// poll() until input available
		do
		{
			int retval = poll(fds, 1, POLL_TIMEOUT);
			if (retval < 0)  // poll() error?
			{
				if (errno != EINTR)
				{
					 printf("[driver][socket] poll() error: %s\n", strerror(errno));
				}
				return 1;
			}
			if (retval == 0)  // poll() timeout?
			{
				printf("[driver][socket] Rslidar poll() timeout\n");

				char buffer_data[8] = "re-con";
				memset(&sender_address, 0, sender_address_len);          // initialize to zeros
				sender_address.sin_family = AF_INET;                     // host byte order
				sender_address.sin_port = htons(MSOP_DATA_PORT_NUMBER);  // port in network byte order, set any value
				sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
				sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
				return 1;
			}
			if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))  // device error?
			{
				printf("[driver][socket] poll() reports Rslidar error");
				return 1;
			}
		} while ((fds[0].revents & POLLIN) == 0);
		ssize_t nbytes = recvfrom(sockfd_, &pkt->data[0], packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

		if (nbytes < 0)
		{
			if (errno != EWOULDBLOCK)
			{
				printf("[driver][socket] recvfail");
				return 1;
			}
		}
		else if ((size_t)nbytes == packet_size)
		{
			if (devip_str_ != "" && sender_address.sin_addr.s_addr != devip_.s_addr)
			{
				continue;
			}
			else
			{
				break;  // done
			}
		}

		printf("[driver][socket] incomplete rslidar packet read:%d bytes\n", nbytes);
	}
	if (flag == 0)
	{
		abort();
	}

	if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
	{//difop
		int rpm = (pkt->data[8]<<8)|pkt->data[9];
		int mode = 1;

		if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08)
				|| (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
		{
			if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
			{
				mode = 0;
			}
		}

		if (cur_rpm_ != rpm || return_mode_ != mode)
		{
			cur_rpm_ = rpm;
			return_mode_ = mode;

			npkt_update_flag_ = true;
		}
	}
	// Average the times at which we begin and end reading.  Use that to
	// estimate when the scan occurred. Add the time offset.
	double time2 = carmen_get_time();
	pkt->stamp = ((time2 + time1) / 2.0 + time_offset);

	return 0;
}

//////////////////////////////////////////////////////////////////////////
//// InputPCAP class implementation
//////////////////////////////////////////////////////////////////////////
//
///** @brief constructor
// *
// *  @param private_nh ROS private handle for calling node.
// *  @param port UDP port number
// *  @param packet_rate expected device packet frequency (Hz)
// *  @param filename PCAP dump file name
// */
//InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
//		bool read_once, bool read_fast, double repeat_delay)
//: Input(private_nh, port), packet_rate_(packet_rate), filename_(filename)
//{
//	pcap_ = NULL;
//	empty_ = true;
//
//	// get parameters using private node handle
//	private_nh.param("read_once", read_once_, false);
//	private_nh.param("read_fast", read_fast_, false);
//	private_nh.param("repeat_delay", repeat_delay_, 0.0);
//
//	if (read_once_)
//	{
//		ROS_INFO("[driver][pcap] Read input file only once.");
//	}
//	if (read_fast_)
//	{
//		ROS_INFO("[driver][pcap] Read input file as quickly as possible.");
//	}
//	if (repeat_delay_ > 0.0)
//	{
//		ROS_INFO("[driver][pcap] Delay %.3f seconds before repeating input file.", repeat_delay_);
//	}
//
//	// Open the PCAP dump file
//	// ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
//	ROS_INFO_STREAM("[driver][pcap] Opening PCAP file " << filename_);
//	if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
//	{
//		ROS_FATAL("[driver][pcap] Error opening rslidar socket dump file.");
//		return;
//	}
//
//	std::stringstream filter;
//	if (devip_str_ != "")  // using specific IP?
//	{
//		filter << "src host " << devip_str_ << " && ";
//	}
//	filter << "udp dst port " << port;
//	int ret = pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
//	if (ret < 0)
//	{
//		ROS_FATAL_STREAM("[driver][pcap] pcap compile filter fail. filter: "<<filter.str());
//		return;
//	}
//}
//
///** destructor */
//InputPCAP::~InputPCAP(void)
//{
//	pcap_close(pcap_);
//}
//
///** @brief Get one rslidar packet. */
//int InputPCAP::getPacket(rslidar_msgs::rslidarPacket* pkt, const double time_offset)
//{
//	struct pcap_pkthdr* header;
//	const u_char* pkt_data;
//
//	while (flag == 1)
//	{
//		int res;
//		if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
//		{
//			// Skip packets not for the correct port and from the
//			// selected IP address.
//			if (!devip_str_.empty() && (0 == pcap_offline_filter(&pcap_packet_filter_, header, pkt_data)))
//				continue;
//
//			// Keep the reader from blowing through the file.
//			if (read_fast_ == false)
//				packet_rate_.sleep();
//
//
//			memcpy(&pkt->data[0], pkt_data + 42, packet_size);
//
//			if (pkt->data[0] == 0xA5 && pkt->data[1] == 0xFF && pkt->data[2] == 0x00 && pkt->data[3] == 0x5A)
//			{//difop
//				int rpm = (pkt->data[8]<<8)|pkt->data[9];
//				int mode = 1;
//
//				if ((pkt->data[45] == 0x08 && pkt->data[46] == 0x02 && pkt->data[47] >= 0x09) || (pkt->data[45] > 0x08)
//						|| (pkt->data[45] == 0x08 && pkt->data[46] > 0x02))
//				{
//					if (pkt->data[300] != 0x01 && pkt->data[300] != 0x02)
//					{
//						mode = 0;
//					}
//				}
//
//				if (cur_rpm_ != rpm || return_mode_ != mode)
//				{
//					cur_rpm_ = rpm;
//					return_mode_ = mode;
//
//					npkt_update_flag_ = true;
//				}
//			}
//
//			pkt->stamp = ros::Time::now();  // time_offset not considered here, as no
//			// synchronization required
//			empty_ = false;
//			return 0;  // success
//		}
//
//		if (empty_)  // no data in file?
//		{
//			ROS_ERROR("[driver][pcap] Error %d reading rslidar packet: %s", res, pcap_geterr(pcap_));
//			return -1;
//		}
//
//		if (read_once_)
//		{
//			ROS_INFO("[driver][pcap] end of file reached -- done reading.");
//			return -1;
//		}
//
//		if (repeat_delay_ > 0.0)
//		{
//			ROS_INFO("[driver][pcap] end of file reached -- delaying %.3f seconds.", repeat_delay_);
//			usleep(rint(repeat_delay_ * 1000000.0));
//		}
//
//		ROS_INFO("[driver][pcap] replaying rslidar dump file");
//
//		// I can't figure out how to rewind the file, because it
//		// starts with some kind of header.  So, close the file
//		// and reopen it with pcap.
//		pcap_close(pcap_);
//		pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
//		empty_ = true;  // maybe the file disappeared?
//	}                 // loop back and try again
//
//	if (flag == 0)
//	{
//		abort();
//	}
//}
}
