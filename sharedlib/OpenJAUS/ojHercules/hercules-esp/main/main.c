#include "system.h"
#include "can.h"
#include "control.h"
#include "freertos/task.h"
#include "odom.h"
#include "test.h"

static const char* TAG = "Main module";

SemaphoreHandle_t odomLeftVelocityMutex = NULL;
SemaphoreHandle_t odomRightVelocityMutex = NULL;
SemaphoreHandle_t odomSteeringMutex = NULL;
SemaphoreHandle_t commandVelocityMutex = NULL;
SemaphoreHandle_t commandSteeringMutex = NULL;
SemaphoreHandle_t commandStepMotorMutex = NULL;
double odom_left_velocity = 0;
double odom_right_velocity = 0;
int odom_steering = 0;
int command_velocity = 0;
int command_steering = 0;
int command_step_motor = 0;

int
create_mutexes ()
{
    // odomVelocityMutex = xSemaphoreCreateMutex ();
    odomLeftVelocityMutex = xSemaphoreCreateMutex ();
    odomRightVelocityMutex = xSemaphoreCreateMutex ();
    odomSteeringMutex = xSemaphoreCreateMutex ();
    commandVelocityMutex = xSemaphoreCreateMutex ();
    commandSteeringMutex = xSemaphoreCreateMutex ();
    commandStepMotorMutex = xSemaphoreCreateMutex ();
    if (odomLeftVelocityMutex == NULL || odomRightVelocityMutex == NULL || odomSteeringMutex == NULL ||
        commandVelocityMutex == NULL || commandSteeringMutex == NULL || commandStepMotorMutex == NULL)
        {
            ESP_LOGE (TAG, "Failed to create mutexes");
            return 0;
        }
    return 1;
}

void
app_main ()
{
    if (!create_mutexes ())
       {
           return;
       }
    if (!can_setup ())
       {
           printf ("Failed to setup CAN\n");
           return;
       }
    
    // Testing tasks
    // xTaskCreate (test_task, "Test Task", 2048, NULL, 1, NULL);
    // xTaskCreate (fake_odometry_task, "Fake Odometry Task",
    //              8192, NULL, 1,
    //              NULL);
    // xTaskCreate (fake_commands_task, "Fake Commands Task",
    //             8192, NULL, 1,
    //             NULL);
    // xTaskCreate (fake_step_motor_task, "Fake Step Motor Task", 8192,
    //             NULL, 1, NULL);

    // Communication
    // xTaskCreate (can_reading_task, "CAN Reading Task",
    //              8192, NULL , 1,
    //              NULL);
    // xTaskCreate (can_writing_task, "CAN Writing Task",
    //              8192, NULL, 1,
    //              NULL);

    // Control
    // xTaskCreate (motor_task, "Motor Task", 8192,
    //         NULL, 1, NULL);
    // xTaskCreate (servo_task, "Servo Task", 8192,
    //            NULL, 1, NULL);
    xTaskCreate (step_motor_task, "Step Motor Task", 8192,
                NULL, 1, NULL);

    // Odometry
    // xTaskCreate (right_encoder_task, "R Encoder Task", 1024 * 8,
    //          NULL, 1, NULL);
    // xTaskCreate (left_encoder_task, "L Encoder Task", 1024 * 8,
    //          NULL, 1, NULL);
    // xTaskCreate (steering_reading_task, "Steering Reading Task",
    //             8192, NULL, 1,
    //            NULL);
        
}
