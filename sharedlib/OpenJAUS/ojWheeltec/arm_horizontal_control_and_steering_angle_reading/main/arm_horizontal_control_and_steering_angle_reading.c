/* UART asynchronous example, that uses separate RX and TX tasks

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include "driver/adc_common.h"
#include "esp_adc_cal.h"

// static const int RX_BUF_SIZE = 1024;

// #define TXD_PIN (GPIO_NUM_4)
// #define RXD_PIN (GPIO_NUM_5)
// #define TXD_PIN (GPIO_NUM_1)
// #define RXD_PIN (GPIO_NUM_3)

//ADC Channels
#define ADC1_EXAMPLE_CHAN0          ADC1_CHANNEL_7

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_11

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static const char *TAG = "ADC SINGLE";
static esp_adc_cal_characteristics_t adc1_chars;


static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) 
    {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } 
    else if (ret == ESP_ERR_INVALID_VERSION) 
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } 
    else if (ret == ESP_OK) 
    {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } 
    else 
    {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}


// void init(void) 
// {
//     const uart_config_t uart_config = 
//     {
//         .baud_rate = 115200,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };
//     // We won't use a buffer for sending data.
//     uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART_NUM_1, &uart_config);
//     uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
// }

// int sendData(const char* logName, const char* data)
// {
//     const int len = strlen(data);
//     const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
//     printf("Wrote %d bytes", txBytes);
//     return txBytes;
// }

// static void tx_task(void *arg)
// {
//     static const char *TX_TASK_TAG = "TX_TASK";
//     esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
//     while (1) 
//     {
//         sendData(TX_TASK_TAG, "Hello world\n");
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

// static void rx_task(void *arg)
// {
//     static const char *RX_TASK_TAG = "RX_TASK";
//     esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
//     uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
//     while (1) 
//     {
//         const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
//         if (rxBytes > 0) 
//         {
//             data[rxBytes] = 0;
//             printf("Read %d bytes: '%s'", rxBytes, data);
//             // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
//         }
//     }
//     free(data);
// }

static void 
adc1_task(void *arg)
{
    uint32_t voltage = 0;
    while (1) 
    {
        for (int i = 0; i < 200; i++)
        {
            uint32_t adc_reading = adc1_get_raw(ADC1_EXAMPLE_CHAN0);
            voltage += esp_adc_cal_raw_to_voltage(adc_reading, &adc1_chars);
            vTaskDelay(0);
        }
        voltage /= 200;
        printf("Voltage: %dmV\n", voltage);
        fflush(stdout);
        vTaskDelay((1000 / 40) / portTICK_PERIOD_MS);
        voltage = 0;
    }
}

static void 
motor_task(void *arg)
{
    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000/40));
    }
}

void app_main(void)
{
    // init();
    // xTaskCreate(rx_task, "uart_rx_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    // xTaskCreate(tx_task, "uart_tx_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);

    adc_calibration_init();

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    xTaskCreatePinnedToCore(adc1_task, "adc1_task", 1024*2, NULL, configMAX_PRIORITIES, NULL, 0);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 1024*2, NULL, configMAX_PRIORITIES, NULL, 1);

    while (1) 
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
