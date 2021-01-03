#include <stdio.h>
#include <carmen/carmen.h>
#include <carmen/can_dump_interface.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <can_utils.h>

int out_can_sockfd = -1;

//Variables for button presses
unsigned char pins[4] = {3, 4, 5, 6};
unsigned char buf[8];
bool flag1 = true;
bool flag2 = true;
bool flag3 = true;
bool flag4 = true;
long lastdebouncetime = 0;
long debouncedelay = 300;
unsigned char buttonno[4] = {1, 2, 3, 4};
unsigned char butt = 0;
int inc = 0;

//CAN Message Lengths
int len2 = 3;
int len3 = 4;
int len4 = 3;
int len5 = 3;
int len6 = 4;
int len7 = 68;

//CAN Addresses and Data
unsigned int array_1[] = {0x423};
unsigned int array_2[] = {0x367, 0x394, 0x3d3};
unsigned int array_3[] = {0x228, 0x351, 0x3bb, 0xba};
unsigned int array_4[] = {0x262, 0x2e4, 0x3e6};
unsigned int array_5[] = {0x1aa, 0x384, 0x386};
unsigned int array_6[] = {0x283, 0x365, 0x366, 0x3e7};
unsigned int array_7[] = {0x24, /*0x25, 0xaa, 0xb4, */0x1c4, 0x1d0,0x1d2, 0x1d3, 0x223, 0x224,
		0x260, 0x2c1, 0x320, 0x343, 0x344, 0x380, 0x381, 0x389, 0x38f, 0x399,
		0x3a5, 0x3b0, 0x3b1, 0x3b7, 0x3bc, 0x3e8, 0x3e9, 0x3f9, 0x411, 0x412,
		0x413, 0x414, 0x420, 0x45a, 0x489, 0x48a, 0x48b, 0x4ac, 0x4cb, 0x4d3,
		0x4ff, 0x610, 0x611, 0x614, 0x615, 0x619, 0x61a, 0x620, 0x621, 0x622,
		0x623, 0x624, 0x638, 0x63c, 0x63d, 0x640, 0x680, 0x6f3, 0x770, 0x778,
		0x7c6, 0x7ce, 0x7e0, 0x7e1, 0x7e2, 0x7e3, 0x7e4, 0x7e5, 0x7e6, 0x7e7,
		0x7e8};
//0x1d2 byte 7, 1st 4 bits(!=0) = cruise_state 0x1d3 byte 2, bit 1 (1) = main_on
//0xaa = wheel speed
//0x3bc = GEAR_PACKET
//0xb4 = SPEED

unsigned char data1[1] = {0x00};
unsigned char data2[2] = {0x0, 0x0};
unsigned char data3[4] = {0x0, 0x0, 0x0, 0x0};
unsigned char data4[5] = {0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data5[6] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data6[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
unsigned char data7[8] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

unsigned char enable1[] = {0xf8, 0x24, 0x02, 0xf8, 0x00, 0x01, 0x80, 0x72};
unsigned char enable2[] = {0x00, 0xa8, 0x43, 0x10, 0xee, 0x00, 0x00, 0xc5};
unsigned char not_in_d[] = {0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

double t0;


void
send_can_message(int can_id, int data_start, int can_dlc, unsigned char *data)
{
	struct can_frame frame;

	frame.can_id = can_id;
	frame.can_dlc = can_dlc;

	for (int i = data_start; i < frame.can_dlc + data_start; i++)
		frame.data[i] = data[i];

	send_frame(out_can_sockfd, &frame);
}


void
delay(int milisseconds)
{
	usleep(milisseconds * 1000);
}


int
millis()
{
	return ((int) (1000.0 * carmen_get_time() - t0));
}


void
loop()
{
	//Temporary ints
	unsigned int temp2 = 0;
	unsigned int temp3 = 0;
	unsigned int temp4 = 0;
	unsigned int temp5 = 0;
	unsigned int temp6 = 0;
	unsigned int temp7 = 0;

	unsigned int spd = 0;
	uint8_t w2 = spd & 0xff;
	uint8_t w1 = (spd >> 8);
	unsigned char wheelpot[] = { w1, w2, w1, w2, w1, w2, w1, w2 };
	unsigned char speedpak[] = { 0x0, 0x0, 0x0, 0x0, w1, w2, 0x0 };

	//Send Pot as Wheelspeed

//	send_can_message(0x423, 0, 1, data1);
	for (int i = 0; i < len2; i++)
	{
		delay(100);
		temp2 = array_2[i];
		send_can_message(temp2, 0, 2, data2);
	}

	for (int i = 0; i < len3; i++)
	{
		delay(50);
		temp3 = array_3[i];
		send_can_message(temp3, 0, 4, data3);
	}

	for (int i = 0; i < len4; i++)
	{
		delay(30);
		temp4 = array_4[i];
		send_can_message(temp4, 0, 5, data4);
	}

	for (int i = 0; i < len5; i++)
	{
		delay(20);
		temp5 = array_5[i];
		send_can_message(temp5, 0, 6, data5);
	}

	for (int i = 0; i < len6; i++)
	{
		delay(10);
		temp6 = array_6[i];
		send_can_message(temp6, 0, 7, data6);
	}

	for (int i = 0; i < len7; i++)
	{
		temp7 = array_7[i];
		send_can_message(temp7, 0, 8, data7);
		send_can_message(0xaa, 0, 8, wheelpot);
		send_can_message(0xb4, 0, 8, speedpak);
		delay(10);
	}

	//Send data if button was pushed
	for (inc = 0; inc <= 3; inc++)
	{
		if ((millis() - lastdebouncetime) > debouncedelay)
		{
			if (1) //digitalRead(pins[inc]) == HIGH)
			{
				lastdebouncetime = millis();
				butt = buttonno[inc];
				if (butt == 1)
				{
					flag1 = !flag1;
				}
				if (butt == 2)
				{
					flag2 = !flag2;
				}
				if (butt == 3)
				{
					flag3 = !flag3;
				}
				if (butt == 4)
				{
					flag4 = !flag4;
				}
			}
			//Send default values if button off
//			else
//			{
//				butt == 0;
//			}
		}
	}
	if (flag1 == true)
	{
		send_can_message(0x1d1, 0, 8, enable1);
	}
	else
	{
		send_can_message(0x1d1, 0, 8, data7);
	}
	if (flag2 == true && flag1 == true)
	{
		send_can_message(0x1d2, 0, 8, enable2);
	}
	else
	{
		send_can_message(0x1d2, 0, 8, data7);
	}
	if (flag3 == true)
	{
		send_can_message(0x3bc, 0, 8, data7);
	}
	else
	{
		send_can_message(0x3bc, 0, 8, not_in_d);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
//	carmen_ipc_initialize(argc, argv);
//	carmen_param_check_version(argv[0]);

	if (argc < 2)
	{
		printf("Error\n Usage: toyota_corolla_simulator <output can interface>\n");
		exit(1);
	}

	out_can_sockfd = init_can(argv[1]);
	if (out_can_sockfd == -1)
	{
		printf("Error: Could not open can%s\n", argv[2]);
		exit(1);
	}

	t0 = carmen_get_time();
	while (1)
	{
		loop();
		delay(10);
	}

	return (0);
}
