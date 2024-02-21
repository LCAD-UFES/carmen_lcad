#include "system.h"
#include "can.h"
#include "control.h"
#include "freertos/task.h"
#include "odom.h"

static const char* TAG = "Main module";
SemaphoreHandle_t odomVelocityMutex = NULL;
SemaphoreHandle_t odomSteeringMutex = NULL;
SemaphoreHandle_t commandVelocityMutex = NULL;
SemaphoreHandle_t commandSteeringMutex = NULL;
int odom_velocity = 0;
int odom_steering = 0;
int command_velocity = 0;
int command_steering = 0;

void
test_task ()
{
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount ();
    const TickType_t xFrequency = can_reading_task_parameters.frequency;
    while (1)
        {
            ESP_LOGI (TAG, "Hello World");
            vTaskDelayUntil (&xLastWakeTime, xFrequency);
        }
}

void
create_mutexes ()
{
    odomVelocityMutex = xSemaphoreCreateMutex ();
    odomSteeringMutex = xSemaphoreCreateMutex ();
    commandVelocityMutex = xSemaphoreCreateMutex ();
    commandSteeringMutex = xSemaphoreCreateMutex ();
    if (odomVelocityMutex == NULL || odomSteeringMutex == NULL ||
        commandVelocityMutex == NULL || commandSteeringMutex == NULL)
        {
            ESP_LOGE (TAG, "Failed to create mutexes");
        }
}

void
app_main ()
{
    create_mutexes ();
    if (!can_setup ())
        {
            printf ("Failed to setup CAN\n");
            return;
        }
    
    // xTaskCreate (test_task, "Test Task", 2048, NULL, 1, NULL);

    // Communication
    xTaskCreate (can_reading_task, "CAN Reading Task",
                 8192, &can_reading_task_parameters, 1,
                 NULL);
    xTaskCreate (can_writing_task, "CAN Writing Task",
                 8192, &can_writing_task_parameters, 1,
                 NULL);

    // // Control
    // xTaskCreate (motor_task, "Motor Task", configMINIMAL_STACK_SIZE,
    //              &motor_task_parameters, 2, NULL);
    // xTaskCreate (servo_task, "Servo Task", configMINIMAL_STACK_SIZE,
    //              &servo_task_parameters, 3, NULL);

    // // Odometry
    // xTaskCreate (encoder_task, "Encoder Task", configMINIMAL_STACK_SIZE,
    //              &encoder_task_parameters, 3, NULL);
    // xTaskCreate (steering_reading, "Steering Reading Task",
    //              configMINIMAL_STACK_SIZE, &steering_reading_parameters, 3,
    //              NULL);
    xTaskCreate (fake_odometry_task, "Fake Odometry Task",
                 8192, &fake_odometry_task_parameters, 1,
                 NULL);    
}
