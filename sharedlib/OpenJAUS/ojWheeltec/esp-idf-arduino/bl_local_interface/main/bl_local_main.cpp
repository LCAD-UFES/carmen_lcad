/*
 * Teremos dois modulos: bl_remote_interface e bl_local_interface.
 *
 * A bl_remote_interface possui um conjunto de botões, e LEDs associados a cada botão, e fica com o operador remoto.
 * A bl_local_interface possui uma interface CAN e fica ligado fisicamente ao hardware do veículo.
 *
 * A bl_remote_interface aguarda o operador apertar um botão. Quando o operador aperta um botão, é enviada uma mensagem
 * para o bl_local_interface informando qual botão foi apertado. Ele também recebe mensagens comandando o acendimento
 * ou o apagamente dos LEDs associados aos botões.
 *
 * Em modo teste, a bl_local_interface, ao receber uma mensagem de botão verde, envia uma mensagem de volta para a bl_remote_interface
 * informando que ela deve acender o LED associado ao botão verde.
 *
 * Em modo teste, a bl_local_interface, ao receber uma mensagem de botão vermelho, envia uma mensagem de volta para a bl_remote_interface
 * informando que ela deve apagar o LED associado ao botão verde.
 */

#define ESP32
#define WIFI_LoRa_32_V2

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "Arduino.h"
#include "heltec.h"
#include "images.h"

#define BL_LOCAL_ID	0
#define BAND    (915E6 + (int) (BL_LOCAL_ID / 2) * 125E3)  //you can set band here directly,e.g. 868E6,915E6

#define WHITE_LED				25

#define EMERGENCY_STOP_RELAY	23 // GPIO number

#define USB_SERIAL_BUFFER_SIZE 	200


String rssi = "RSSI --";
String packSize = "--";
String packet;

unsigned int counter = 0;
bool receiveflag = false; // software flag for LoRa receiver, received data makes it true.
uint64_t chipid;
char leds[5];

int emergency_stop_button = 0;
int green_button = 0;
int red_button = 0;
int black_button = 0;


void logo()
{
	Heltec.display->clear();
	Heltec.display->drawXbm(0, 5, logo_width, logo_height, (const unsigned char *) logo_bits);
	Heltec.display->display();
}

void send_leds(int bl_interface_id, char *leds)
{
	char message[100];

	sprintf(message, "id: %d, leds: %s", bl_interface_id, leds);

	LoRa.beginPacket();
	LoRa.print(message);
	LoRa.endPacket();

	counter++;
}

void displaySendReceive()
{
	Heltec.display->drawString(0, 0, "Received Size  " + packSize + " package:");
	Heltec.display->drawString(0, 10, packet);
	Heltec.display->drawString(0, 20, "With " + rssi + "db");
	Heltec.display->drawString(0, 50, "Last Packet sent: " + (String) counter);
	Heltec.display->display();
	delay(1);
	Heltec.display->clear();
}

void clear_leds()
{
	sprintf(leds, "0000");
}

void onReceive(int packetSize)          // LoRa receiver interrupt service
{
	packet = "";
	packSize = String(packetSize, DEC);

	while (LoRa.available())
		packet += (char) LoRa.read();

	rssi = "RSSI: " + String(LoRa.packetRssi(), DEC);
	receiveflag = true;
}

void setup()
{
	Heltec.begin(true /*DisplayEnable Enable*/, true /*LoRa Enable*/,
			true /*Serial Enable*/, true /*LoRa use PABOOST*/,
			BAND /*LoRa RF working band*/);

	logo();
	delay(300);
	Heltec.display->clear();

	chipid = ESP.getEfuseMac(); //The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32ChipID=%04X", (uint16_t) (chipid >> 32)); //print High 2 bytes
	Serial.printf("%08X\n", (uint32_t) chipid); //print Low 4bytes.

	// Config LoRa
	LoRa.setTxPower(17, RF_PACONFIG_PASELECT_PABOOST); 	// Potência do trasmissor
//	LoRa.setSpreadingFactor(12); 						// Fator de espalhemento entre 6 e 12
//	LoRa.setCodingRate4(8); 							// denominador da taxa de codificação entre 5 e 8
	LoRa.setSignalBandwidth(125E3); 					// Largura de banda do sinal padrão 125E3
	LoRa.enableCrc();
	// End Config LoRa

	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, HIGH);

	pinMode(EMERGENCY_STOP_RELAY, OUTPUT);
	digitalWrite(EMERGENCY_STOP_RELAY, HIGH);

	LoRa.onReceive(onReceive);

	clear_leds();
//	send();
	displaySendReceive();

	LoRa.receive();
}

void update_buttons_local_state(String packet)
{
	String search_pattern = "buttons: ";
	int buttons_state = packet.indexOf(search_pattern) + search_pattern.length();

	if (packet.charAt(buttons_state + 0) == '1')
		emergency_stop_button = 1;
	else
		emergency_stop_button = 0;

	if (packet.charAt(buttons_state + 1) == '1')
		green_button = 1;
	else
		green_button = 0;

	if (packet.charAt(buttons_state + 2) == '1')
		red_button = 1;
	else
		red_button = 0;

	if (packet.charAt(buttons_state + 3) == '1')
		black_button = 1;
	else
		black_button = 0;
}

void
send_message_to_pc(char *message)
{
	printf("$ %s *\n", message);
}

static void
receive_from_lora(void *not_used)
{
	while (1)
	{
		if (receiveflag)
		{
			receiveflag = false;

			send_message_to_pc((char *) packet.c_str());

			displaySendReceive();

			update_buttons_local_state(packet);

			if (emergency_stop_button)
				digitalWrite(EMERGENCY_STOP_RELAY, LOW);
			else
				digitalWrite(EMERGENCY_STOP_RELAY, HIGH);

			LoRa.receive();
		}

		delay(10);
	}
}

static void
read_from_USB(void *v_buffer)
{
	int i = 0;

	char *buffer = (char *) v_buffer;

	while (1)
	{
		uint8_t ch = fgetc(stdin);

		if ((ch == '$') || (i >= USB_SERIAL_BUFFER_SIZE))
			i = 0;

		if (ch != 0XFF)
		{
			buffer[i] = ch;
			i++;
		}

		delay(10);
	}
}

void
clear_buffer(char *buffer)
{
	for (int i = 0; i < USB_SERIAL_BUFFER_SIZE; i++)
		buffer[i] = '\0';
}

extern "C" void app_main()
{
	initArduino();

	setup();

	char buffer[USB_SERIAL_BUFFER_SIZE];
	buffer[0] = '\0';
    xTaskCreatePinnedToCore(read_from_USB, "read_from_USB", 4096, buffer, 5, NULL, 1);

    xTaskCreatePinnedToCore(receive_from_lora, "receive_from_lora", 4096, NULL, 5, NULL, 0);

    while (true)
	{
		// Release the core for 10ms (necesary to avoid wdt (watchdog timer) complains).
		delay(20);
		char *end_of_leds_message_received = strchr(buffer, '*');
		if (end_of_leds_message_received)
		{
			end_of_leds_message_received[0] = '\0';

			if (strstr(buffer, "initializing"))
			{
				send_message_to_pc(buffer);
				clear_buffer(buffer);
				emergency_stop_button = 0;
			}
			else if (strstr(buffer, "leds"))
			{
				int bl_interface_id;
				char leds_received[10];
				sscanf(buffer, "$ id: %d, leds: %s", &bl_interface_id, leds_received);
				for (int i = 0; i < 4; i++)
					leds[i] = leds_received[i];

				delay(1200);
				LoRa.idle();
				send_leds(bl_interface_id, leds);
				LoRa.receive();
			}
			else if (strstr(buffer, "emergency_stop"))
			{
				int bl_interface_id = 0;
				char *leds_received = "1111"; // Acende o led zero (emeregency) e os demais da interface zero
				for (int i = 0; i < 4; i++)
					leds[i] = leds_received[i];

				delay(1200);
				LoRa.idle();
				send_leds(bl_interface_id, leds);
				LoRa.receive();
			}
		}
	}
}
