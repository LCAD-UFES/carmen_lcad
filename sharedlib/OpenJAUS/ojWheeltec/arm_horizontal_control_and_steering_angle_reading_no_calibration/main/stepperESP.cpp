#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"

#include "Arduino.h"

/* Enable ativa os drivers do motor */
#define ENA 13
#define END 23
#define STP 33 /* Avanço do passo */
#define DIR 32 /* Direção do passo  */

#define FRAME_HEADER 0XFF
#define FRAME_TAIL 0XFE
#define DATA_LENGTH 5

#define STEPS_PER_REV 200
#define NUM_REV_0_TO_100 24.0

void set_steering_effort(int steering_effort);

const double percent_to_steps = NUM_REV_0_TO_100 / 300.0;

byte dataReceived[DATA_LENGTH];
unsigned short receivedValue = 0;
int new_linear_actuator_command = 0;
int currentSteps = 0;
int numSteps;

char data_string[1024];


void HR()
{ /* Sentido Horário */
    // Serial.println("Sentido - Horario");
    digitalWrite(DIR, LOW);
}


void AH()
{ /* Sentido Anti-Horário */
    // Serial.println("Sentido - Anti-horario");
    digitalWrite(DIR, HIGH);
}


void linearActuator()
{
    numSteps = receivedValue * percent_to_steps - currentSteps;
    if (numSteps >= 0)
        AH();
    else
        HR();
    digitalWrite(ENA, LOW);
    for (int i = 0; i < abs(numSteps); i++)
    {
        digitalWrite(STP, LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
        digitalWrite(STP, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    digitalWrite(ENA, HIGH);
    currentSteps += numSteps;
}


unsigned char checkSum(unsigned char Count_Number, byte *data)
{
    unsigned char check_sum = 0, k;

    // Validate the data
    for (k = 0; k < Count_Number; k++)
    {
        check_sum = check_sum ^ data[k];
    }

    return check_sum;
}


void calibrate()
{
    HR();
    digitalWrite(ENA, LOW);
    while (1) // digitalRead(END) == LOW)
    {
        digitalWrite(STP, LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
        digitalWrite(STP, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
//    receivedValue = 30000;
//    linearActuator();
//    digitalWrite(ENA, HIGH);
}


void step_motor_setup()
{
    pinMode(ENA, OUTPUT);
    pinMode(STP, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(END, INPUT_PULLUP);
    digitalWrite(ENA, HIGH);
    Serial.begin(115200);
}

void motor_task(void *arg)
{
    // calibrate();

    while (1)
    {
        digitalWrite(ENA, HIGH);
        if (new_linear_actuator_command == 1)
        {
        	linearActuator();
            new_linear_actuator_command = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void serial_task(void *arg)
{
    // calibrate();

    while (1)
    {
        if (Serial.available())	// Tem que colocar esta leitura em uma task e as variaveis receivedValue e steering_effort (dentro de set_steering_effort()) protegidas por semaforos
        {
            int data_string_size = Serial.readBytesUntil('\n', data_string, 1023);
            data_string[data_string_size] = '\0';
            int steering_effort;
            if (strstr(data_string, "steering_effort: ") != NULL)
            {
                if (sscanf(data_string, "steering_effort: %d", &steering_effort) == 1) // Checa se recebeu steering_effort
                    set_steering_effort(steering_effort);
            }

            if (strstr(data_string, "linear_actuator_percent: ") != NULL)
            {
                if ((new_linear_actuator_command == 0) && (sscanf(data_string, "linear_actuator_percent: %hu", &receivedValue) == 1)) // Checa se recebeu linear_actuator_percent
                	new_linear_actuator_command = 1;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
