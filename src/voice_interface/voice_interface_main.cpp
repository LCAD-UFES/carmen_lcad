#include <stdio.h>
#include <iostream>
#include <carmen/carmen.h>
#include <carmen/voice_interface_interface.h>
#include <alsa/asoundlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <curl/curl.h>
#include <jsoncpp/json/json.h>
#include "voice_interface.h"
#include "porcupine_keyword.h"

using namespace std;

extern snd_pcm_t* capture_handle;


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Publishers                                                                                //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
publish_voice_interface_can_line_message(char *can_line)
{
//	carmen_voice_interface_can_line_message message;

	can_line[strlen(can_line) - 1] = '\0'; // Apaga o '\n' no fim da string

	FILE *caco = fopen("voice_interface.txt", "a");
	fprintf(caco, "%lf can_line %s\n", carmen_get_time(), can_line);
	fflush(caco);
	fclose(caco);

//	message.can_line = can_line;
//	message.timestamp = carmen_get_time();
//	message.host = carmen_get_host();

//	carmen_voice_interface_publish_can_line_message(&message);
}
///////////////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
shutdown_module(int signo)
{
	if (signo == SIGINT)
	{
		printf("voice interface: disconnected.\n");
		finalize_voice();
		finalize_porcupine();

		exit(0);
	}
}
///////////////////////////////////////////////////////////////////////////////////////////////


static size_t
curlopt_writefunction(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string *) userp)->append((char *) contents, size * nmemb);
    return (size * nmemb);
}


Json::Value
get_rasa_server_response(char *query)
{
	CURL *curl;
	CURLcode curl_error;
	string http_server_string_response;

	curl = curl_easy_init();
	if (curl)
	{
		char *http_query = curl_easy_escape(curl, query, strlen(query));
		if (http_query)
		{
//			printf("Encoded: %s\n", http_query);
			char http_query_post[2048];
			strncpy(http_query_post, "http://localhost:5000/parse?q=", 2048);
			strncat(http_query_post, http_query, 2048 - (strlen(http_query_post) + strlen(http_query)));
			strncat(http_query_post, "&project=current&model=nlu", 2048 - (strlen(http_query_post) + strlen("&project=current&model=nlu")));
//			printf("Encoded POST: %s\n", http_query_post);
			curl_easy_setopt(curl, CURLOPT_URL, http_query_post);
			curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlopt_writefunction);
			curl_easy_setopt(curl, CURLOPT_WRITEDATA, &http_server_string_response);
			curl_error = curl_easy_perform(curl);
			if (curl_error)
			{
				printf("Error in check_command(). Could not get curl http response in check_command().\n");
				exit(1);
			}

			curl_easy_cleanup(curl);
			curl_free(http_query);
		}
		else
		{
			printf("Error in check_command(). Could not get http_query in check_command().\n");
			exit(1);
		}

//		std::cout << readBuffer << endl;

		stringstream input_string_stream(http_server_string_response);
		Json::Reader reader;
		Json::Value http_server_json_response;
		reader.parse(input_string_stream, http_server_json_response);
		cout << http_server_json_response << std::endl;
//		cout << "Project: " << http_server_json_response["project"].asString() << endl;
//		cout << "Entities[0]: " << http_server_json_response["entities"][0] << endl;
//		cout << "Entities:entity: " << http_server_json_response["entities"][0]["entity"].asString() << endl;
//		cout << "Intent: " << http_server_json_response["intent"] << endl;
//		cout << "Intent: " << http_server_json_response["intent"]["name"].asString() << endl;

		return (http_server_json_response);
	}
	else
	{
		printf("Error in check_command(). Could not get curl object.\n");
		exit(1);
	}
}


void
execute_voice_command(char *voice_command)
{
	if (voice_command)
	{
		printf("\nVoice command: %s \n", voice_command);
		if (strcmp(voice_command, "timeout") == 0)
		{
			char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Desculpe... Não consegui captar o comando.");
			if (voice_interface_speak_error)
				printf("%s \n", voice_interface_speak_error);
		}
		else
		{
			Json::Value rasa_server_response = get_rasa_server_response(voice_command);
			printf("rasa_server_response[\"intent\"][\"confidence\"] %lf\n", rasa_server_response["intent"]["confidence"].asDouble());
			if (rasa_server_response["intent"]["confidence"].asDouble() > 0.7)
			{
				if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "engage") == 0)
				{
					printf("Command detected: %s \n\n", "Seguir curso");
					carmen_navigator_ackerman_go();
					carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
					system("mpg123 $CARMEN_HOME/data/voice_interface_hotword_data/helm_engage_clean.mp3"); // http://www.trekcore.com/audio/
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "set_course") == 0)
				{
					printf("Command detected: %s \n\n", "Estabelecer curso");
					bool place_detected = false;
					for (unsigned int i = 0; i < rasa_server_response["entities"].size(); i++)
					{
						cout << "Entity :" << rasa_server_response["entities"][i]["entity"].asString() << endl;
						if (strcmp(rasa_server_response["entities"][i]["entity"].asString().c_str(), "place") == 0)
						{
							cout << "Entity value la vai:" << endl;
							cout << "Entity value :" << rasa_server_response["entities"][i]["value"].asString() << endl;
							char *voice_interface_speak_error = carmen_voice_interface_speak((char *) ("Curso para " +
									rasa_server_response["entities"][i]["value"].asString() + " definido.").c_str());
							if (voice_interface_speak_error)
								printf("%s \n", voice_interface_speak_error);

							place_detected = true;
						}
					}
					if (!place_detected)
					{
						char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Desculpe... Não identifiquei nenhum lugar de meu conhecimento em seu comando...\n");
						if (voice_interface_speak_error)
							printf("%s \n", voice_interface_speak_error);
						voice_interface_speak_error = carmen_voice_interface_speak((char *) ("Você disse " + rasa_server_response["text"].asString() + "?\n").c_str());
						if (voice_interface_speak_error)
							printf("%s \n", voice_interface_speak_error);
						voice_interface_speak_error = carmen_voice_interface_speak((char *) ("Se o que você disse está correto, favor incluir o lugar mencionado em minha base de dados."));
						if (voice_interface_speak_error)
							printf("%s \n", voice_interface_speak_error);
					}
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "greet") == 0)
				{
					printf("Command detected: %s \n\n", "Olá");
					char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Olá!... Bom ter você aqui!");
					if (voice_interface_speak_error)
						printf("%s \n", voice_interface_speak_error);
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "thankyou") == 0)
				{
					printf("Command detected: %s \n\n", "Obrigada");
					char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Por nada!");
					if (voice_interface_speak_error)
						printf("%s \n", voice_interface_speak_error);
				}
				else
				{
					char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Desculpe... Sua inteção parace clara, mas não sei o que fazer...\n");
					if (voice_interface_speak_error)
						printf("%s \n", voice_interface_speak_error);
					voice_interface_speak_error = carmen_voice_interface_speak((char *) ("Você disse " + rasa_server_response["text"].asString() + "?\n").c_str());
					if (voice_interface_speak_error)
						printf("%s \n", voice_interface_speak_error);
				}
			}
			else
			{
				char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Desculpe... Não entendi claramente sua intenção...\n");
				if (voice_interface_speak_error)
					printf("%s \n", voice_interface_speak_error);
				voice_interface_speak_error = carmen_voice_interface_speak((char *) ("Você disse " + rasa_server_response["text"].asString() + "?\n").c_str());
				if (voice_interface_speak_error)
					printf("%s \n", voice_interface_speak_error);
			}
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Initializations                                                                           //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////


void
voice_interface_define_messages()
{
	carmen_voice_interface_define_can_line_message();
}


void
carmen_voice_interface_initialize()
{
	char *voice_interface_error = init_voice();
	if (voice_interface_error != NULL)
	{
		printf("Error: could not initialize the voice interface.\n%s\n", voice_interface_error);
		exit(1);
	}

	char *porcupine_error = initialize_porcupine();
	if (porcupine_error != NULL)
	{
		printf("Error: could not initialize porcupine.\n%s\n", porcupine_error);
		exit(1);
	}
}


int
main (int argc, char **argv)
{
	carmen_ipc_initialize(argc, argv);
//	carmen_param_check_version(argv[0]);
	signal(SIGINT, shutdown_module);
	voice_interface_define_messages();
	carmen_voice_interface_initialize();

//	char *voice_interface_speak_error = carmen_voice_interface_speak((char *) "Oi Alberto!");
//	if (voice_interface_speak_error)
//		printf("%s \n", voice_interface_speak_error);

	printf("Awaiting hotword\n");
	while (true)
	{
		int hotword_detection_result = hotword_detection();
		if (hotword_detection_result == 1) // hotword detected
		{
			snd_pcm_drop(capture_handle);

			printf("Hotword detected\n");

			carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
			system("mpg123 $CARMEN_HOME/data/voice_interface_hotword_data/computerbeep_4.mp3"); // http://www.trekcore.com/audio/

			printf("Awaiting for command\n\n");
			char *voice_command = carmen_voice_interface_listen();
			execute_voice_command(voice_command);

			snd_pcm_prepare(capture_handle);
			snd_pcm_start(capture_handle);

			printf("Awaiting hotword\n");
		}
		else if (hotword_detection_result == 2) // error
			printf ("Error in hotword detection\n");

		carmen_ipc_sleep(0.0);
	}

	return (0);
}
