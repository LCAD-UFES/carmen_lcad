#include "wifi_and_tcp.c"

void clean_str(char* string)
{
    for (int i = 2; i < strlen(string); i++)
    {
        string[i-2] = string[i];
    }

}

void tcp_user(int* sock, int* connected)
{

    extern QueueHandle_t queue_servo;
    char rx_buffer[128];

    while (*connected == CONNECTED) {

            // Envia a mensagem
            int err = send(*sock, payload, strlen(payload), 0);
            if (err < 0) {
                ESP_LOGE(TAG_EXAMPLE, "Error occurred during sending: errno %d", errno);
                *connected = DISCONNECTED;;
            }

            // Recebe a resposta
            int len = recv(*sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            if (len < 0) {
                ESP_LOGE(TAG_EXAMPLE, "recv failed: errno %d", errno);
                *connected = DISCONNECTED;
            }

            // Processamento da mensagem (envia para queue)
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                // ESP_LOGI(TAG_EXAMPLE, "Received %d bytes from %s:", len, HOST_IP_ADDR);
                // ESP_LOGI(TAG_EXAMPLE, "%s", rx_buffer);
                if (rx_buffer[0] == '0')
                {
                    clean_str(rx_buffer);
                    xQueueSend(queue_servo, rx_buffer, portMAX_DELAY);
                }

                

            }
        }

}

void manage_messages()
{

    extern QueueHandle_t queue_servo;
    
    int connected = CONNECTED;

    struct sockaddr_in dest_addr;
    int addr_family = 0;
    int ip_protocol = 0;

    while ((1))
    {

        // Connecta ao wifi
        connect_to_wifi();

        // Faz a conexão TCP
        inet_pton(AF_INET, HOST_IP_ADDR, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        // Update the Connected status
        connected = CONNECTED;

        // Cria socket
        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG_EXAMPLE, "Unable to create socket: errno %d", errno);
            connected = DISCONNECTED;
        }
        //ESP_LOGI(TAG_EXAMPLE, "Socket created, connecting to %s:%d", HOST_IP_ADDR, PORT);

        // Conecta
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if ((err != 0) & (connected == CONNECTED)) {
            ESP_LOGE(TAG_EXAMPLE, "Socket unable to connect: errno %d", errno);
            connected = DISCONNECTED;;
        }
        else
        {
                    //ESP_LOGI(TAG_EXAMPLE, "Successfully connected");
        }

        /* TCP_USER DEFINE COMO SERÁ UTILIZADA A CONEXÃO */
        tcp_user(&sock,&connected);

        if (sock != -1) {
            ESP_LOGE(TAG_EXAMPLE, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelay(DELAY_TIME_TRYING_TO_CONNECT / portTICK_PERIOD_MS);




    }
    

    

}