#include "includes.c"
#include "config.c"

void config_servo_pin()
{
    // ESP_LOGI("Config", "Setting up pins...");
    
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

     // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
    };
    ledc_channel_config(&ledc_channel);
}

void manage_servo()
{

    config_servo_pin();
    extern QueueHandle_t queue_servo;
    int duty_cycle = 0;

    while (1)
    {
        char msg[128];

        if (xQueueReceive(queue_servo, msg, portMAX_DELAY)) {
            int duty = atoi(msg);
            duty_cycle = (int)((20.47)*duty);    
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_cycle);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        }

    }
    
}