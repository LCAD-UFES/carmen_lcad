#include "ADS1256_DAC8235.h"


#define MAX_ANALOG_EFFORT_VALUE 3.1


void
program_termintaion_handler(int signo)
{
	printf("Program exited...\n");

	digitalWrite(SIGNAL_RELAY_PIN, LOW);
	DAC8532_Out_Voltage(channel_A, 0.0);

    DEV_ModuleExit();

    exit(0);
}


int
test_board(void)
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
        for(i=0;i<8;i++){
            printf("%d %f\r\n",i,ADC[i]*5.0/0x7fffff);
        }

        x = (ADC[0] >> 7)*5.0/0xffff;
        printf(" %f \r\n", x);
        printf("\33[9A");//Move the cursor up 8 lines
        DAC8532_Out_Voltage(channel_B, (x));
        DAC8532_Out_Voltage(channel_A, (3.3 - x));

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
set_on_off_wire_state(double analog_pedal_signal)
{
	static int pin_state = 0;

	if (analog_pedal_signal > 0.1 && pin_state == 0)
	{
		digitalWrite(SIGNAL_RELAY_PIN, HIGH);
		pin_state = 1;
	}
	else if (analog_pedal_signal <= 0.1 && pin_state == 1)
	{
		digitalWrite(SIGNAL_RELAY_PIN, LOW);
		pin_state = 0;
	}
}


int
main(void)
{
	UDOUBLE ADC_pedal_value_read;
	double effort = 0.0, analog_pedal_signal = 0.0;
	char key;

	signal(SIGINT, program_termintaion_handler);

	wiringPiSetup();
	pinMode(SIGNAL_RELAY_PIN, OUTPUT);
	DEV_ModuleInit();

	while(1)
	{
		key = getchar();

		if (key == 'w' && effort <= 100)
			effort += 5;
		else if (key == 's' && effort >= 0)
			effort -= 5;

		effort = check_effort_range(effort);

		analog_pedal_signal = (effort * MAX_ANALOG_EFFORT_VALUE) / 100;

		set_on_off_wire_state(analog_pedal_signal);

		DAC8532_Out_Voltage(channel_A, analog_pedal_signal);
		printf ("Effort: %f\n", analog_pedal_signal);

		delay(500);
	}

	return 0;
}

