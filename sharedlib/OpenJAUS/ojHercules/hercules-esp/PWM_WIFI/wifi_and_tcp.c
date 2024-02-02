#include "includes.c"
#include "config.c"

/** Variables **/

// Counter of number of tries
int s_retry_num =0;

// event group to contain status information
static EventGroupHandle_t wifi_event_group;

/** Functions **/

/* WIFI */
//event handler for wifi events
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		//ESP_LOGI(TAG_WIFI, "Connecting to AP...");
		esp_wifi_connect();
	} else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		if (s_retry_num < MAX_FAILURES)
		{
			//ESP_LOGI(TAG_WIFI, "Reconnecting to AP...");
			esp_wifi_connect();
			s_retry_num++;
		} else {
			xEventGroupSetBits(wifi_event_group, WIFI_FAILURE);
		}
	}
}

//event handler for ip events
static void ip_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
	if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        //ESP_LOGI(TAG_WIFI, "STA IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_SUCCESS);
    }

}

// connect to wifi 
void connect_to_wifi()
{
	esp_err_t status = WIFI_FAILURE;

    //initialize storage
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /** INITIALIZE ALL THE THINGS **/
    //initialize the esp network interface
    ESP_ERROR_CHECK(esp_netif_init());

    //initialize default esp event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    //create wifi station in the wifi driver
    esp_netif_create_default_wifi_sta();

    //setup wifi station with the default wifi configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    while(status != WIFI_SUCCESS)
    {

        /** EVENT LOOP CRAZINESS **/
        wifi_event_group = xEventGroupCreate();

    

        esp_event_handler_instance_t wifi_handler_event_instance;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                            ESP_EVENT_ANY_ID,
                                                            &wifi_event_handler,
                                                            NULL,
                                                            &wifi_handler_event_instance));

        esp_event_handler_instance_t got_ip_event_instance;
        ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                            IP_EVENT_STA_GOT_IP,
                                                            &ip_event_handler,
                                                            NULL,
                                                            &got_ip_event_instance));

        /** START THE WIFI DRIVER **/
        wifi_config_t wifi_config = {
            .sta = {
                .ssid = SSID,
                .password = PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
                .pmf_cfg = {
                    .capable = true,
                    .required = false
                },
            },
        };

        // set the wifi controller to be a station
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

        // set the wifi config
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );

        // start the wifi driver
        ESP_ERROR_CHECK(esp_wifi_start());

        //ESP_LOGI(TAG_WIFI, "STA initialization complete");

        /** NOW WE WAIT **/
        EventBits_t bits = xEventGroupWaitBits(wifi_event_group,
                WIFI_SUCCESS | WIFI_FAILURE,
                pdFALSE,
                pdFALSE,
                portMAX_DELAY);

        /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
        * happened. */
        if (bits & WIFI_SUCCESS) {
            //ESP_LOGI(TAG_WIFI, "Connected to ap");
            status = WIFI_SUCCESS;
        } else if (bits & WIFI_FAILURE) {
            //ESP_LOGI(TAG_WIFI, "Failed to connect to ap");
            status = WIFI_FAILURE;
        } else {
            ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
            status = WIFI_FAILURE;
        }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, got_ip_event_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_event_instance));
    vEventGroupDelete(wifi_event_group);

    }
}