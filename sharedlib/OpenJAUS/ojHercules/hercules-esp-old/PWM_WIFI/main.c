#include <stdio.h>
#include "task_wifi_communication.c"
#include "servo_controller.c"

QueueHandle_t queue_servo;

void app_main(void)
{

    queue_servo = xQueueCreate(5, 5*sizeof(float)); // Queue 

    xTaskCreatePinnedToCore(manage_messages, "Manage mesages", 1024*4, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(manage_servo, "Manage Servo", 1024*2, NULL, 1, NULL, 1);

}