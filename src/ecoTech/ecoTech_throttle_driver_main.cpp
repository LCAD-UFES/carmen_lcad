#include "ADS1256_DAC8235.h"
#include "relay.h"
#include <ncurses.h>

// gcc -o test ecoTech_throttle_driver_main.c ADS1256_DAC8235.c ADS1256_DAC8235.h relay.c relay.h -l wiringPi -lbcm2835 -lm

/*
#define MAX_ANALOG_EFFORT_VALUE 3.1           // Value that corresponds to 5v (the signal that is sent to emulate the pedal)

unsigned int autonomous_mode = 0;

void
program_termintaion_handler(int signo)
{
	printf("Program exited...\n");

	digitalWrite(SIGNAL_RELAY_PIN, LOW);
	DAC8532_Out_Voltage(DAC0, 0.0);

    DEV_ModuleExit();

    exit(0);
}


int
test_board(void)            // Change the led's color based on the potentiometer and print all the 8 AD input values
{
    UDOUBLE ADC[8], i;
    float x;

    printf("demo\r\n");
    DEV_ModuleInit();

    signal(SIGINT, program_termintaion_handler);

    if(ADS1256_init() == 1)
    {
        printf("\r\nEND                  \r\n");
        DEV_ModuleExit();
        exit(0);
    }

    while(1)
    {
        ADS1256_GetAll(ADC);
        for(i = 0; i < 8; i++)
        {
            printf("%d %f\r\n",i,ADC[i]*5.0/0x7fffff);
        }

        x = (ADC[0] >> 7)*5.0/0xffff;
        printf(" %f \r\n", x);
        printf("\33[9A");//Move the cursor up 8 lines
        DAC8532_Out_Voltage(DAC1, (x));
        DAC8532_Out_Voltage(DAC0, (3.3 - x));

    }
    return 0;
}


double
check_effort_range(double effort)
{
	if (effort < 0.0)
		return 0.0;
	else if (effort > 100.0)
		return 100.0;

	return effort;
}


void
set_pedal_switch_state(double analog_pedal_signal)
{
	static int pin_state = 0;

	if (analog_pedal_signal > 0.1 && pin_state == 0)
	{
		//digitalWrite(SIGNAL_RELAY_PIN, HIGH);
		set_relay_state(0, 1, ON);
		pin_state = 1;
	}
	else if (analog_pedal_signal <= 0.1 && pin_state == 1)
	{
		//digitalWrite(SIGNAL_RELAY_PIN, LOW);
		set_relay_state(0, 1, OFF);
		pin_state = 0;
	}
}


void
update_effort_using_keyboard(double &effort)
{
	switch (getch())      // getch() reads the keyboard and return an int value
	{
	case KEY_UP:
		effort += 5;
		break;
	case KEY_DOWN:
		effort -= 5;
		break;
	}
}


void
print_test_values(double analog_pedal_signal)
{
	clear();
	printw ("-------------------------------------\n");
	printw ("    e.coTech Trottle Driver Test     \n");
	printw ("-------------------------------------\n\n");
	printw ("Effort: %f\n", analog_pedal_signal);
}


void
initialize_board ()
{
	wiringPiSetup();
//	pinMode(SIGNAL_RELAY_PIN, OUTPUT);
//	pinMode(CONTROL_ON_OFF_RELAY_PIN, OUTPUT);
//	pinMode(SIGNAL_ON_OFF_RELAY_PIN, OUTPUT);
	DEV_ModuleInit();

	printf ("AD/DA Board Initialized Successfully!\n");
}


int
main(int argc, char *argv [])
{
	UDOUBLE ADC_pedal_value_read;
	double effort = 0.0, analog_pedal_signal = 0.0, signal_diff = 0.0;
	unsigned int test_mode = 1, autonomous_state = 0;

	signal(SIGINT, program_termintaion_handler);

	initialize_board ();

	initscr();
	keypad(stdscr, TRUE);              // Allow dealing with special keys (arrows...)
	noecho();
	refresh();

	while(1)
	{
		effort = check_effort_range(effort);
		//effort = carmen_clamp(0.0, effort, 100.0);

		analog_pedal_signal = (effort * MAX_ANALOG_EFFORT_VALUE) / 100;

		if (autonomous_mode == autonomous_state)
		{
			if (test_mode)
				update_effort_using_keyboard(effort);

			//		}
			//		else
			//		{
			//			effort =    ; // TODO receber o effort do carmen
			//		}

			set_pedal_switch_state(analog_pedal_signal);

			DAC8532_Out_Voltage(DAC0, analog_pedal_signal);
		}
		else
		{
			ADC_pedal_value_read = ADS1256_GetChannalValue(0);
			signal_diff = abs(ADC_pedal_value_read - analog_pedal_signal);

			double step = (MAX_ANALOG_EFFORT_VALUE / 10) * ((ADC_pedal_value_read - analog_pedal_signal) / (ADC_pedal_value_read - analog_pedal_signal));

			while (signal_diff > 0.0)
			{
				double pedal_value = ADC_pedal_value_read + step;

				DAC8532_Out_Voltage(DAC0, pedal_value);
				signal_diff += step;

				usleep(1000);
			}
			autonomous_state = autonomous_mode;
		}

	}
	endwin();

	return 0;
}*/



int
main(int argc, char *argv [])
{
	wiringPiSetup();
	pinMode(29, INPUT);          // configura pino 7 como entrada
	pullUpDnControl(29, PUD_UP); // configura resistor pull-up no pino 7

	while(1)
	{
		if(digitalRead(29) == LOW)
		{
			printf("Autonomous\n");
		}
		else
		{
			printf("Manual\n");
		}
		usleep(100000);
	}

	return 0;
}
