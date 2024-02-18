#include "can.h"
#include "control.h"
#include "freertos/task.h"
#include "odom.h"
#include "system.h"

void
create_mutexes ()
{
    odomVelocityMutex = xSemaphoreCreateMutex ();
    odomSteeringMutex = xSemaphoreCreateMutex ();
    commandVelocityMutex = xSemaphoreCreateMutex ();
    commandSteeringMutex = xSemaphoreCreateMutex ();
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

    // Communication
    xTaskCreate (can_reading_task, "CAN Reading Task",
                 configMINIMAL_STACK_SIZE, &can_reading_task_parameters, 1,
                 NULL);
    xTaskCreate (can_writing_task, "CAN Writing Task",
                 configMINIMAL_STACK_SIZE, &can_writing_task_parameters, 1,
                 NULL);

    // Control
    xTaskCreate (motor_task, "Motor Task", configMINIMAL_STACK_SIZE,
                 &motor_task_parameters, 2, NULL);
    xTaskCreate (servo_task, "Servo Task", configMINIMAL_STACK_SIZE,
                 &servo_task_parameters, 3, NULL);

    // Odometry
    xTaskCreate (encoder_task, "Encoder Task", configMINIMAL_STACK_SIZE,
                 &encoder_task_parameters, 3, NULL);
    xTaskCreate (steering_reading, "Steering Reading Task",
                 configMINIMAL_STACK_SIZE, &steering_reading_parameters, 3,
                 NULL);

    // Start FreeRTOS scheduler
    vTaskStartScheduler ();
    // while (1)
    // {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}
