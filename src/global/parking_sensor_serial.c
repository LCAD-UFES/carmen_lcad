#include <stdio.h>
 #include <sys/time.h>
#include <carmen/carmen.h>
#include "carmenserial.h"

int fd;
int bit_chain_count = 0;
char bit_chain[45];
char ordered_bit_chain[45];
float sensor_value[4];

void
cleanup(int signo)
{
  if (signo == SIGINT)
  {
  	carmen_serial_close(fd);
    exit(0);
  }
}

int main()
{
	int i, j, k;
	register int cts_c = 0, cts_l = 0;
	register double diff_time = 0;
	register double t1, t2;
	char* device_name = "/dev/ttyUSB0";
	struct timeval tim;

	signal(SIGINT, cleanup);

	if(!carmen_serial_connect(&fd, device_name))
	{
		carmen_serial_configure(fd, 500000, "O");

		gettimeofday(&tim, NULL);
		t1 = (tim.tv_usec / 1000.0);

		while (1)
		{
			cts_l = cts_c;
			cts_c = carmen_serial_read_cts(fd);

			gettimeofday(&tim, NULL);
			t2 = (tim.tv_usec / 1000.0);

			diff_time = t2 - t1;

			if(diff_time > 2.20 && diff_time <= 12.00)	// otherwise is reset signal -1
			{
				if((cts_l == 1 && cts_c == 0) || (cts_l == 0 && cts_c == 1))
				{
					t1 = t2;
					bit_chain[bit_chain_count] = -1;
					bit_chain_count++;
				}
			}
			else
			{
				if (diff_time <= 1.1)			// 1 ms period is equal bit 1
				{
					if(cts_l == 0 && cts_c == 1)  // check if bit changes from 0 to 1
					{
						t1 = t2;
						bit_chain[bit_chain_count] = 1;
						bit_chain_count++;
					}
				}
				else if(diff_time > 1.1 && diff_time <= 2.20)   // 2 ms period is equal bit 0
				{
					if(cts_l == 0 && cts_c == 1)  // check if bit changes from 0 to 1
					{
						t1 = t2;
						bit_chain[bit_chain_count] = 0;
						bit_chain_count++;
					}
				}
				else
					t1 = t2;
			}

			if(bit_chain_count == 45)
			{
				int number_of_minus_one = 0;

				for(i = 0; i < bit_chain_count; i++)
				{
					if(bit_chain[i] == -1)
						number_of_minus_one++;
				}

				if(number_of_minus_one == 3)
				{
					for(i = 0; i < bit_chain_count; i++)
					{
						if(bit_chain[i] == -1)
						{
							for(j = i, k=0; j < (i + bit_chain_count); j++, k++)
							{
								ordered_bit_chain[k] = bit_chain[j%bit_chain_count];
							}
							break;
						}
					}

					j = 0;
					for(i=3; i < bit_chain_count-10; i+=8)
					{
						if(ordered_bit_chain[i] == 1 && ordered_bit_chain[i+1] == 1 && ordered_bit_chain[i+2] == 1)
						{
							int value = 0;

							value |= ((!ordered_bit_chain[i+3]) << 4);
							value |= ((!ordered_bit_chain[i+4]) << 3);
							value |= ((!ordered_bit_chain[i+5]) << 2);
							value |= ((!ordered_bit_chain[i+6]) << 1);
							value |= ((!ordered_bit_chain[i+7]) << 0);

							sensor_value[j] = value / 10.0;
							j++;
						}
					}

					printf("1: %6.2f, 2: %6.2f, 3: %6.2f, 4: %6.2f \n	", sensor_value[0], sensor_value[1], sensor_value[2], sensor_value[3]);

				}

				bit_chain_count = 0;
			}

		}
	}
	else
	{
		printf("cannot open device!\n");
	}

	return 0;
}
