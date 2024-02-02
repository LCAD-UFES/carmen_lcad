
// Host Config
#define HOST_IP_ADDR "192.168.0.159"
#define PORT 12345

// Wifi Config
#define SSID "LCAD04-02"
#define PASSWORD "1q2w3e4r"

/** General Defines **/

// Wifi and TCP 
#define WIFI_SUCCESS 1 << 0
#define WIFI_FAILURE 1 << 1
#define TCP_SUCCESS 1 << 0
#define TCP_FAILURE 1 << 1
#define MAX_FAILURES 10
#define CONNECTED 1
#define DISCONNECTED 0

// Tags
#define TAG_WIFI "WIFI"
#define TAG_EXAMPLE "Example"
#define payload "Duty_Cycle (0-100)"

// Delay for reconnecting TCP
#define DELAY_TIME_TRYING_TO_CONNECT 5000 // em ms

// PWM
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (LED_pin) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_11_BIT // Set duty resolution to 11 bits
#define LEDC_FREQUENCY          (18000) // Frequency in Hertz. Set frequency at 5 kHz
#define LED_pin 27 // led
// #define LEDC_DUTY               (1027) // Set duty to 50%. ((2 ** 11) - 1) * 50% = 1027

#define PERIOD 1/LEDC_FREQUENCY // em ms
//#define MIN_T_HIGH 1500 // em ms
//#define MAX_T_HIGH 3500 // em ms
 