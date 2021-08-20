#include "lidar_drivers.h"

#define RSLIDAR_MSG_BUFFER_SIZE 1248

#define POLLIN		0x001		// There is data to read
#define POLLPRI		0x002		// There is urgent data to read
#define POLLOUT		0x004		// Writing now will not block


int sockfd_;


void
initizalize_socket_connection(uint16_t port)
{
	sockfd_ = -1;
	
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
	if (sockfd_ == -1)
	{
		printf("Create socket fail\n");
		return;
	}

	int opt = 1;
	if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
	{
		printf("Setsockopt fail\n");
		return;
	}

	sockaddr_in my_addr;                   // my address information
	memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
	my_addr.sin_family = AF_INET;          // host byte order
	my_addr.sin_port = htons(port);        // port in network byte order
	my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

	if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
	{
		printf("Socket bind fail\n");
		return;
	}

	if (fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC) < 0)
	{
		printf("fcntl fail\n");
		return;
	}
}


int
receive_packet_from_socket(uint8_t *socket_data, uint16_t port, char *ip)
{
	in_addr devip_;
	struct pollfd fds[1];
	int received_packet_size = 0;
	fds[0].fd = sockfd_;
	fds[0].events = POLLIN;
	static const int POLL_TIMEOUT = 3000;
	size_t packet_size = RSLIDAR_MSG_BUFFER_SIZE;
	ssize_t nbytes;

	sockaddr_in sender_address;
	socklen_t sender_address_len = sizeof(sender_address);
	
    while (true)
	{
		do
		{
			int retval = poll(fds, 1, POLL_TIMEOUT);   // Wait for some event on a file descriptor
			if (retval < 0)  // Error
			{
				if (errno != EINTR)
				{
					 printf("poll() error: %s\n", strerror(errno));
				}
				return 1;
			}
			if (retval == 0)  // Timeout
			{
				printf("Rslidar poll() timeout\n");

				char buffer_data[8] = "re-con";
				memset(&sender_address, 0, sender_address_len);          // initialize to zeros
				sender_address.sin_family = AF_INET;                     // host byte order
				sender_address.sin_port = htons(port);                   // port in network byte order, set any value
                inet_aton(ip, &devip_);                                  // inet_aton(ip_adress_string, *sender_address.sin_addr.s_addr)
				sender_address.sin_addr.s_addr = devip_.s_addr;          // automatically fill in my IP
				sendto(sockfd_, &buffer_data, strlen(buffer_data), 0, (sockaddr*)&sender_address, sender_address_len);
				return 1;
			}
			if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))
			{
				printf("poll() reports Rslidar error\n");
				return 1;
			}
		} while ((fds[0].revents & POLLIN) == 0);

		nbytes = recvfrom(sockfd_, socket_data, packet_size, 0, (sockaddr*)&sender_address, &sender_address_len);

		if (nbytes < 0)
		{
			if (errno != EWOULDBLOCK)
			{
				printf("[driver][socket] recvfail");
				return 1;
			}
		}
		else if ((size_t)nbytes == packet_size)
			break;  // done

		if (socket_data[0] != 0x55 || socket_data[1] != 0xAA || socket_data[2] != 0x05 || socket_data[3] != 0x0A)     // check pkt header and returns if not a packt of LiDAR measures
		{
			return 1;
		}
		printf("[driver][socket] incomplete rslidar packet read:%ld bytes\n", nbytes);
	}
	received_packet_size = nbytes;

	if (received_packet_size < 1248)
		printf ("packet_size %d\n", received_packet_size);

	return (0);
}


carmen_velodyne_shot*
realloc_message(carmen_velodyne_variable_scan_message &msg, int actual_size)
{
	msg.partial_scan = (carmen_velodyne_shot *) realloc (msg.partial_scan, (actual_size + 240) * sizeof(carmen_velodyne_shot));
	
	actual_size = actual_size - 1;
    
	for (int i = actual_size; i < (actual_size + 240); i++)
	{
		// TODO Should free the memory before alloc new block?
		msg.partial_scan[i].shot_size = msg.partial_scan[actual_size].shot_size;
		msg.partial_scan[i].distance = (unsigned int*) malloc (msg.partial_scan[actual_size].shot_size * sizeof(unsigned int));
		msg.partial_scan[i].intensity = (short unsigned int*) malloc (msg.partial_scan[actual_size].shot_size * sizeof(short unsigned int));
	}
	return (msg.partial_scan);
}


bool
unpack_socket_data(carmen_velodyne_variable_scan_message &msg, int *num_shots, const uint8_t *socket_data, carmen_velodyne_shot *shots_array)
{
	static int max_num_shot = INITIAL_MAX_NUM_SHOT;
	int last_byte_of_shot;
	bool complete_turn = false;
	carmen_velodyne_shot* current_shot = &shots_array[*num_shots];

	for (int i = 44; i < 1242; i += 100) // i = 44 to skip the header (42b) and the first data block identifier (2b) 0xffee
	{
		current_shot->angle = (double)((256 * socket_data[i] + socket_data[i + 1]) / 100.0);

		if (*num_shots > 0 && current_shot->angle < 5.0 && shots_array[*num_shots - 2].angle > 355.0)
			complete_turn = true;

		last_byte_of_shot = i + 2 + 48; // skip 2 bytes of azimuth and 48 bytes of measures (the 50th byte is the last byte of first channel)

		for (int index = i + 2, k = 0; index < last_byte_of_shot; index += 3, k++)  // i + 2 to skip the two bytes of the azimuth
		{
            if (socket_data[index] > 120)
			{
				current_shot->distance[k] = 0;
				current_shot->intensity[k] = 0;
				continue;
			}
			current_shot->distance[k] = 256 * socket_data[index] + socket_data[index + 1];
			current_shot->intensity[k] = socket_data[index + 2];
		}

		*num_shots += 1;
		if (*num_shots > max_num_shot)
			shots_array = realloc_message(msg, max_num_shot);

		current_shot = &shots_array[*num_shots];

		current_shot->angle = -1;

        last_byte_of_shot = i + 98;

		for (int index = i + 50, k = 0; index < last_byte_of_shot; index += 3, k++)  // 52 is the first byte of the second channel of the current block and 100 is the last byte of the current block
		{
			if (socket_data[index] > 120)
			{
				current_shot->distance[k] = 0;
				current_shot->intensity[k] = 0;
				continue;
			}
			current_shot->distance[k] = 256 * socket_data[index] + socket_data[index + 1];
            current_shot->intensity[k] = socket_data[index + 2];
		}

        *num_shots += 1;
		if (*num_shots > max_num_shot)
			shots_array = realloc_message(msg, max_num_shot);

		current_shot = &shots_array[*num_shots];
	}
	return (complete_turn);
}


void
fill_and_publish_variable_scan_message(carmen_velodyne_variable_scan_message &msg, carmen_velodyne_shot *shots_array, int *num_shots, int lidar_id)
{
	double previous_angle = 0.0, next_angle;
	int last_shot_of_scan = 0;

	int last_valid_shot = *num_shots - 1;

	for (int i = 1; i < last_valid_shot; i++)
	{
		if (shots_array[i].angle == -1)
		{
			previous_angle = shots_array[i - 1].angle;
			next_angle = shots_array[i + 1].angle;

			if (next_angle < previous_angle)                            // Adjust for a rollover from 359.99 to 0 // TODO verficar se eh necessario
				next_angle = next_angle + 360.0;

			shots_array[i].angle = previous_angle + ((next_angle - previous_angle) / 2.0); // Interpolate

			if (shots_array[i].angle > 360.0)                           // Correct for any rollover over form 359.99 to 0
			 	shots_array[i].angle = shots_array[i].angle - 360.0;
		}
        
		if (*num_shots > 0 && shots_array[i].angle < 5.0 && shots_array[i - 1].angle > 355.0)
        {
			last_shot_of_scan = i;
        }
	}

	msg.number_of_shots = last_shot_of_scan;
	msg.timestamp = carmen_get_time();

	carmen_velodyne_publish_variable_scan_message(&msg, lidar_id);

	for (int i = last_shot_of_scan, j = 0; i < last_valid_shot; i++, j++)
	{
		memcpy(&shots_array[j], &shots_array[i], sizeof(carmen_velodyne_shot));
	}

	*num_shots = *num_shots - last_shot_of_scan;
}


void
run_robosense_RSLiDAR16_driver(carmen_velodyne_variable_scan_message &msg, carmen_lidar_config lidar_config, int lidar_id)
{
	static int num_shots = 0;
    uint8_t socket_data[RSLIDAR_MSG_BUFFER_SIZE];

	uint16_t port = atoi(lidar_config.port);

    initizalize_socket_connection(port);

	while (true)
	{
		while (true)
		{
			int rc = receive_packet_from_socket(socket_data, port, lidar_config.ip);
			if (rc == 0)   // Full packet received
				break;
			if (rc < 0)
				continue;
		}

		if (unpack_socket_data(msg, &num_shots, socket_data, msg.partial_scan))
		{
			fill_and_publish_variable_scan_message(msg, msg.partial_scan, &num_shots, lidar_id);
		}
	}
	printf("\nLidar sensor: disconnected.\n");
}
