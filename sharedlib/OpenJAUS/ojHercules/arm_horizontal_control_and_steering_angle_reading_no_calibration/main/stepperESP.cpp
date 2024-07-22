#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "driver/twai.h"

#include "Arduino.h"


//Constantes ESP32 e Hercules

#define PIN_CAN_TX GPIO_NUM_16
#define PIN_CAN_RX GPIO_NUM_4
#define COMMAND_CAN_CAR_ID 0x100
#define COMMAND_CAN_STEP_MOTOR_ID 0x90
#define ODOM_VELOCITY_CAN_ID 0x425
#define ODOM_STEERING_CAN_ID 0x80

#define CAN_MAX_COMMAND 256*100
#define MAX_VELOCITY 3.0

#define VELOCITY_CONVERSION_CONSTANT (CAN_MAX_COMMAND/MAX_VELOCITY)
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
void set_velocity_command(int velocity_command);

const double percent_to_steps = NUM_REV_0_TO_100 / 300.0;

byte dataReceived[DATA_LENGTH];
unsigned short receivedValue = 0;
int velocity_command;
int new_linear_actuator_command = 0;
int new_velocity_command = 0;
int currentSteps = 0;
int numSteps;

double current_velocity;
int current_steering;
//char data_string[1024];


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

void step_motor_task(void *arg)
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

void motor_task(void *arg)
{
    // calibrate();

    while (1)
    {
        digitalWrite(ENA, HIGH);
        if (new_velocity_command == 1)
        {
        	set_velocity_command(velocity_command);
            new_velocity_command = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// void serial_task(void *arg)
// {
//     // calibrate();

//     while (1)
//     {
//         if (Serial.available())	// Tem que colocar esta leitura em uma task e as variaveis receivedValue e steering_effort (dentro de set_steering_effort()) protegidas por semaforos
//         {
//             int data_string_size = Serial.readBytesUntil('\n', data_string, 1023);
//             data_string[data_string_size] = '\0';
//             int steering_effort;
//             if (strstr(data_string, "steering_effort: ") != NULL)
//             {
//                 if (sscanf(data_string, "steering_effort: %d", &steering_effort) == 1) // Checa se recebeu steering_effort
//                     set_steering_effort(steering_effort);
//             }

//             if (strstr(data_string, "linear_actuator_percent: ") != NULL)
//             {
//                 if ((new_linear_actuator_command == 0) && (sscanf(data_string, "linear_actuator_percent: %hu", &receivedValue) == 1)) // Checa se recebeu linear_actuator_percent
//                 	new_linear_actuator_command = 1;
//             }

//             if (strstr(data_string, "velocity_command: ") != NULL)
//             {
//                 if ((new_velocity_command == 0) && (sscanf(data_string, "velocity_command: %d", &velocity_command) == 1)) // Checa se recebeu linear_actuator_percent
//                 	new_velocity_command = 1;
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

int
can_setup()
{
    printf("Setting up CAN\n");
    //Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX,TWAI_NORMAL_MODE);
    g_config.tx_queue_len = 1;
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    printf("Configurations set\n");

    //Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver\n");
        return 0;
    }

    //Start CAN driver
    if (twai_start() == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver\n");
        return 0;
    }
    
    return 1;
}

void can_reading_task()
{
    //variables for saving the values temporaly
    twai_message_t message;
    int16_t command_velocity_received;
    int16_t command_steering_effort_received;
    int16_t command_step_motor_received;

    if (!can_setup())
    {
        printf("Coudn't setup can\n");
        return;
    }

    while (1)
    {
        //Receive the message
        if (twai_receive(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
        {
            //Check if the message is for the steering effort
            if (message.identifier == COMMAND_CAN_CAR_ID)
            {
                printf("Command received for steering and velocity\n");
                command_velocity_received = (message.data[1] << 8) | message.data[0];
                command_steering_effort_received = (message.data[3] << 8) | message.data[2];
                set_steering_effort(command_steering_effort_received);
            }
            //Check if the message is for the linear actuator
            else if (message.identifier == COMMAND_CAN_STEP_MOTOR_ID)
            {
                // ESP_LOGD (TAG, "Command received for step motor");
                // command_step_motor_received = (message.data[1] << 8) | message.data[0];
                // set_command_step_motor(command_step_motor_received);
                // ESP_LOGD (TAG, "CAN Step Motor command: %hi", command_step_motor);
            
            //Check if the message is for the velocity command
            }
        }
    }
}

int
send_can_message (uint32_t id, double data)
{
    twai_message_t message = {.data_length_code = 2};
    message.identifier = id;
    if (id == ODOM_VELOCITY_CAN_ID)
    {
        int16_t data_send = (int16_t) round(data * VELOCITY_CONVERSION_CONSTANT);
        message.data[0] = (uint8_t) (data_send & 0xFF);
        message.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
    }
    else if (id == ODOM_STEERING_CAN_ID)
    {
        uint16_t data_send = (uint16_t) round(data);
        message.data[0] = (uint8_t) (data_send & 0xFF);
        message.data[1] = (uint8_t) ((data_send >> 8) & 0xFF);
    }
    
    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS (1000)) == ESP_OK)
    {
        printf("Message queued for transmission\n");
        return 1;
    }
    else
    {
       printf("Failed to queue message for transmission\n");
        return 0;
    }
}

void
can_writing_task()
{
    while (1)
    {
        // Send the velocity command
        send_can_message(ODOM_VELOCITY_CAN_ID, velocity_command);
        // Send the steering effort
        send_can_message(ODOM_STEERING_CAN_ID, receivedValue);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}