/** @file
 *
 *  Velodyne 3D LIDAR data input classes
 *
 *    These classes provide raw Velodyne LIDAR input packets from
 *    either a live socket interface or a previously-saved PCAP dump
 *    file.
 *
 *  Classes:
 *
 *     velodyne::Input -- pure virtual base class to access the data
 *                      independently of its source
 *
 *     velodyne::InputSocket -- derived class reads live data from the
 *                      device via a UDP socket
 *
 *     velodyne::InputPCAP -- derived class provides a similar interface
 *                      from a PCAP dump file
 */

#ifndef __VELODYNE2_INPUT_H
#define __VELODYNE2_INPUT_H

#include <unistd.h>
#include <stdio.h>

namespace velodyne_driver
{
//const double VELODYNE_PACKAGE_RATE = 1808.0;
//const double DIST_LSB = 0.2; // Nearest Distance Strip
//const float ROTATION_RESOLUTION = 0.01f; // Degrees
//const unsigned short ROTATION_MAX_UNITS = 36000; // Hundredths of degrees
const int VELODYNE_NUM_LASERS = 32; // The number of lasers per shot
//const int VELODYNE2_NUM_LASERS = 16; // The number of lasers per shot
//const int VELODYNE3_NUM_LASERS = 16; // The number of lasers per shot
const int VELODYNE_NUM_SHOTS = 12; // The number of shots per packet
//const int MIN_SENSING_DISTANCE = 1000; //2m in 2mm units
//const int MAX_SENSING_DISTANCE = 24000; //1200m in 2mm units
const int VELODYNE_MSG_BUFFER_SIZE = 1206; //The sides of a packet
const int VELODYNE_GPS_BUFFER_SIZE = 498;

//const double VELODYNE_DRIVER_BROADCAST_FREQ_HZ = 60.0; //The rate of broadcast packets
//const int VELODYNE_UDP_PORT = 2368; //The port the Velodyne sends to
//const int VELODYNE_GPS_UDP_PORT = 8308; //The port the Velodyne sends to

//const unsigned short VELODYNE_UPPER_HEADER_BYTES = 61183; //The byte indicating a upper shot
//const unsigned short VELODYNE_LOWER_HEADER_BYTES = 56831; //The byte indicating a lower shot

//const double GYRO_SCALE_FACTOR = 0.09766;
//const double TEMP_SCALE_FACTOR = 0.1453;
//const int TEMP_BASE_FACTOR = 25;
//const double ACCEL_SCALE_FACTOR = 0.001221;

//const double VELODYNE_MIN_FREQUENCY = 1.0;
//const int VELODYNE_MAX_32_LASER_SHOTS_PER_REVOLUTION = (int) ((double) (VELODYNE_PACKAGE_RATE * VELODYNE_NUM_SHOTS) * VELODYNE_MIN_FREQUENCY + 0.5);

// Velodyne datastructures
typedef struct vel_laser {
	unsigned short int distance; // 2mm increments
	unsigned char intensity; // 255 being brightest
} __attribute__((packed)) vel_laser_t;

typedef struct velodyne_fire {
	unsigned short int start_id;
	unsigned short int rotational_pos; // 0-35999  divide by 100 for degrees
	vel_laser_t lasers[VELODYNE_NUM_LASERS];
} __attribute__((packed)) velodyne_fire_t ;

typedef struct velodyne_packet {
	velodyne_fire_t shots[VELODYNE_NUM_SHOTS];
	unsigned char status_timestamp[4];
	unsigned char blank_value2;			// not used
	unsigned char blank_value1;			// not used
}  __attribute__((packed))	 velodyne_packet_t ;

typedef struct velodyne_gps_packet {
	unsigned char not_used4[14];
	unsigned short int gyro1;
	unsigned short int temp1;
	unsigned short int accel1_x;
	unsigned short int accel1_y;
	unsigned short int gyro2;
	unsigned short int temp2;
	unsigned short int accel2_x;
	unsigned short int accel2_y;
	unsigned short int gyro3;
	unsigned short int temp3;
	unsigned short int accel3_x;
	unsigned short int accel3_y;
	unsigned char not_used3[160];
	unsigned char gps_timestamp[4];
	unsigned char not_used2[4];
	unsigned char nmea_sentence[72];
	unsigned char not_used1[220];
} __attribute__((packed)) velodyne_gps_packet_t;

/** @brief Pure virtual Velodyne input base class */
class Input
{
public:
	Input() {}

	/** @brief Read one Velodyne packet.
	 *
	 * @param pkt points to VelodynePacket message
	 *
	 * @returns 0 if successful,
	 *          -1 if end of file
	 *          > 0 if incomplete packet (is this possible?)
	 */
	virtual int getScanPacket(velodyne_packet_t *pkt, int velodyne_udp_port) = 0;
	virtual int getGpsPacket(velodyne_gps_packet_t *pkt, int velodyne_gps_udp_port) = 0;
};

/** @brief Live Velodyne input from socket. */
class InputSocket: public Input
{
public:
	InputSocket(unsigned short udp_port);
	~InputSocket();

	virtual int getScanPacket(velodyne_packet_t *pkt, int velodyne_udp_port);
	virtual int getGpsPacket(velodyne_gps_packet_t *pkt, int velodyne_gps_udp_port);
	
	void printGpsPacket(velodyne_gps_packet_t packet);

private:
	int sockfd_;
	int port_number;
};
} // velodyne_driver namespace

#endif // __VELODYNE2_INPUT_H
