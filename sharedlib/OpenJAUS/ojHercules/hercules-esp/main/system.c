#include "system.h"

static const char *TAG = "Error";

/***********           General purpose functions           ***********/
// The following functions are used to update the value of global variables
int 
get_odom_steering ()
{
    int current_odom_steering = 0;

    if (xSemaphoreTake (odomSteeringMutex, 1000 / portTICK_PERIOD_MS))
    {
        current_odom_steering = odom_steering;
        xSemaphoreGive (odomSteeringMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take odom steering mutex");
    }
    
    return current_odom_steering;
}

double 
get_odom_left_velocity ()
{
    double current_odom_left_velocity = 0;

    if (xSemaphoreTake (odomLeftVelocityMutex, 1000 / portTICK_PERIOD_MS))
    {
        current_odom_left_velocity = odom_left_velocity;
        xSemaphoreGive (odomLeftVelocityMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take odom left velocity mutex");
    }

    return current_odom_left_velocity;
}

double 
get_odom_right_velocity ()
{
    double current_odom_right_velocity = 0;

    if (xSemaphoreTake (odomRightVelocityMutex, 1000 / portTICK_PERIOD_MS))
    {
        current_odom_right_velocity = odom_right_velocity;
        xSemaphoreGive (odomRightVelocityMutex);
    }
    else
    {
        ESP_LOGE (TAG, "Failed to take odom right velocity mutex");
    }
    
    return current_odom_right_velocity;
}

int 
get_command_steering ()
{
    int current_command_steering = 0;

    if (xSemaphoreTake (commandSteeringMutex, 1000 / portTICK_PERIOD_MS))
    {
        current_command_steering = command_steering;
        xSemaphoreGive (commandSteeringMutex);
    }
    else
    {
        ESP_LOGE (TAG, "Failed to take command steering mutex");
    }

    return current_command_steering;
}

int 
get_command_velocity ()
{
    int current_command_velocity = 0;

    if (xSemaphoreTake (commandVelocityMutex, 1000 / portTICK_PERIOD_MS))
    {
        current_command_velocity = command_velocity;
        xSemaphoreGive (commandVelocityMutex);
    }
    else
    {
        ESP_LOGE (TAG, "Failed to take command velocity mutex");
    }

    return current_command_velocity;
}

int 
get_command_step_motor ()
{
    int current_command_step_motor = 0;

    if (xSemaphoreTake (commandStepMotorMutex, 1000 / portTICK_PERIOD_MS))
    {
        current_command_step_motor = command_step_motor;
        xSemaphoreGive (commandStepMotorMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take command step motor mutex");
    }

    return current_command_step_motor;
}

// The following functions are used to update the value of global variables
void 
set_odom_steering (int new_odom_steering)
{
    if (xSemaphoreTake (odomSteeringMutex, 1000 / portTICK_PERIOD_MS))
    {
        odom_steering = new_odom_steering;
        xSemaphoreGive (odomSteeringMutex);
    }   
    else
    {
        ESP_LOGE (TAG, "Failed to take odom steering mutex");
    }
}

void 
set_odom_left_velocity (double new_left_velocity)
{
    if (xSemaphoreTake (odomLeftVelocityMutex, 1000 / portTICK_PERIOD_MS))
    {
        odom_left_velocity = new_left_velocity;
        xSemaphoreGive (odomLeftVelocityMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take odom left velocity mutex");
    }
}

void 
set_odom_right_velocity (double new_odom_right_velocity)
{
    if (xSemaphoreTake (odomRightVelocityMutex, 1000 / portTICK_PERIOD_MS))
    {
        odom_right_velocity = new_odom_right_velocity;
        xSemaphoreGive (odomRightVelocityMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take odom right velocity mutex");
    }
}

void 
set_command_velocity (int new_command_velocity)
{
    if (xSemaphoreTake (commandVelocityMutex, 1000 / portTICK_PERIOD_MS))
    {
        command_velocity = new_command_velocity;
        xSemaphoreGive (commandVelocityMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take command velocity mutex");
    }
}

void 
set_command_steering (int new_command_steering)
{
    if (xSemaphoreTake (commandSteeringMutex, 1000 / portTICK_PERIOD_MS))
    {
        command_steering = new_command_steering;
        xSemaphoreGive (commandSteeringMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take command steering mutex");
    }
}

void 
set_command_step_motor (int new_command_step_motor)
{
    if (xSemaphoreTake (commandStepMotorMutex, 1000 / portTICK_PERIOD_MS))
    {
        command_step_motor = new_command_step_motor;
        xSemaphoreGive (commandStepMotorMutex);
    } 
    else
    {
        ESP_LOGE (TAG, "Failed to take command step motor mutex");
    }
}

// The following functions limit a variable to a certain range
double 
target_limit_double (double insert,double low,double high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}