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
SemaphoreHandle_t resetErrorAndAngleCounterMutex = NULL;
SemaphoreHandle_t commandSteeringEffortMutex = NULL;
double odom_left_velocity = 0;
double odom_right_velocity = 0;
int odom_steering = 0;
int command_velocity = 0;
int command_steering = 0;
int command_steering_effort = 0;
int command_step_motor = 0;
char reset_error_and_angle_counter = 0;

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
    resetErrorAndAngleCounterMutex = xSemaphoreCreateMutex ();
    commandSteeringEffortMutex = xSemaphoreCreateMutex ();

    if (odomLeftVelocityMutex == NULL || odomRightVelocityMutex == NULL || odomSteeringMutex == NULL
        || commandVelocityMutex == NULL || commandSteeringMutex == NULL || commandStepMotorMutex == NULL 
        || resetErrorAndAngleCounterMutex == NULL || commandSteeringEffortMutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return 0;
    }
    
    return 1;
}

void
app_main ()
{
    if (!create_mutexes ())
    {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return;
    }
    
    // Testing tasks
    // xTaskCreate(test_task, "Test Task", 2048, NULL, 1, NULL);
    // xTaskCreate(fake_odometry_task, "Fake Odometry Task", 8192, NULL, 1, NULL);
    // xTaskCreate(fake_commands_task, "Fake Commands Task", 8192, NULL, 1, NULL);
    // xTaskCreate(fake_step_motor_task, "Fake Step Motor Task", 8192, NULL, 1, NULL);
    // xTaskCreate(measure_encoder_task, "Measure Encoder Task", 8192, NULL, 1, NULL);

    // Communication
    xTaskCreate(can_reading_task, "CAN Reading Task", 8192, NULL, 1, NULL);
    xTaskCreate(can_writing_task, "CAN Writing Task", 8192, NULL, 1, NULL);

    // Control
    xTaskCreate(motor_task, "Motor Task", 8192, NULL, 1, NULL);
    xTaskCreate(servo_task, "Servo Task", 8192, NULL, 1, NULL);
    xTaskCreate(step_motor_task, "Step Motor Task", 8192, NULL, 1, NULL);
    xTaskCreate(reset_error_and_angle_task, "Reset E/A Task", 8192, NULL, 1, NULL);

    // Odometry
    xTaskCreate(left_encoder_task, "L Encoder Task", 8192, NULL, 1, NULL);
    xTaskCreate(right_encoder_task, "R Encoder Task", 8192, NULL, 1, NULL);
    xTaskCreate(steering_reading_task, "Steering Reading Task", 8192, NULL, 1, NULL);
}
