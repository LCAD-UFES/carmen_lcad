#include <stdio.h>
#include <stdlib.h>


int
main (int argc, char **argv)
{
	FILE *sent_raw_data = fopen(argv[1], "r");
	int sensor = atoi(argv[2]);

	int state = -1;
	int value;
	char line[1024];
	while (fgets(line, 1023, sent_raw_data))
	{
		int nibble;
		if (sscanf(line, "%d\n", &nibble) != 1)
		{
			state = 0;
			value = 0;
		}
		else
			state++;

		if (sensor == 1)
		{
			if ((state > 1) && (state <= 4))
				value = (value << 4) | (nibble & 0xf);

			if (state == 5)
			{
//				value -= 750;
				if (value > 2047)
					value = value - 4096;

				printf("%d\n", value);
			}
		}
		else if (sensor == 2)
		{
			if ((state > 4) && (state <= 7))
			{
//				printf("nibble = 0x%x\n", nibble);
				value = (value << 4) | (nibble & 0xf);
			}

			if (state == 8)
			{
				value = ((value << 8) & 0xf00) | (value & 0xf0) | ((value >> 8) & 0xf); // troca a ordem dos nibbles
//				value += 750;
				if (value > 2047)
					value = value - 4096;

				printf("%d\n", value);
//				printf("0x%x\n", value);
			}
		}
		else	// sensor == 3 -> status nibble
		{
			if (state == 1)
			{
				static int first_time = 1;
				static int status_info = 0;

				if (((nibble & 0xf) == 8) && first_time)
				{
					first_time = 0;
				}
				else if ((nibble & 0xf) == 8)
				{
					printf("0x%x\n", status_info);
					status_info = 0;
				}
				else
					status_info = (status_info << 1) | ((nibble & 0xf) / 4);
			}
		}
	}
}
