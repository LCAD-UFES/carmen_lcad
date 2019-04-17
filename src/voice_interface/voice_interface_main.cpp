#include <strings.h>
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
#include <locale.h>
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
publish_voice_interface_command_message(const char *command, int command_id)
{
	carmen_voice_interface_command_message message;

	message.command_id = command_id;
	message.command = (char *) command;
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	carmen_voice_interface_publish_command_message(&message);
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
speek_sentence(char *sentence)
{
	char *voice_interface_speak_error = carmen_voice_interface_speak(sentence);
	if (voice_interface_speak_error)
		printf("%s \n", voice_interface_speak_error);
}


char *
check_if_place_is_known(const char *place)
{
	char places_list_database[2048];

	char *carmen_home = getenv("CARMEN_HOME");
	if (carmen_home == NULL)
	{
		printf("CARMEN_HOME not defined in start_porcupine()\n");
		return (NULL);
	}

	strcpy(places_list_database, carmen_home);
	strcat(places_list_database, "/data/voice_interface_data/places_list_database.txt");
	FILE *places = fopen(places_list_database, "r");
	if (places)
	{
		char a_place[1024];
		char *place_in_database;
		char *rddf;
		while (fgets(a_place, 1023, places) != NULL)
		{
			if ((a_place[0] == '#') || (a_place[0] == '\n') || (a_place[0] == ' ')) // se o primeiro caracter for um destes a linha eh descartada
				continue;

			place_in_database = strtok(a_place, ":");
			rddf = strtok(NULL, ": ");
			if (rddf[strlen(rddf) - 1] == '\n')
				rddf[strlen(rddf) - 1] = '\0';

			printf("************ %s\n", rddf);
			if (strcasecmp(place, (const char *) place_in_database) == 0)
			{
				static char rddf_file_name[2048];
				strcpy(rddf_file_name, rddf);

				return (rddf_file_name);
			}
		}
		return (NULL);
	}
	else
	{
		speek_sentence((char *) "Erro de sistema! Não consegui abrir a base de dados de lugares conhecidos");
		return (NULL);
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
			speek_sentence((char *) "Desculpe... Não consegui captar o comando.");
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

					publish_voice_interface_command_message("MAX_SPEED", SET_SPEED);
					carmen_navigator_ackerman_go();

					carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
					system("mpg123 $CARMEN_HOME/data/voice_interface_data/helm_engage_clean.mp3"); // http://www.trekcore.com/audio/
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "initialize") == 0)
				{
					printf("Command detected: %s \n\n", "Inicializar");

					carmen_navigator_ackerman_stop();
					publish_voice_interface_command_message("MAX_SPEED", SET_SPEED);

					speek_sentence((char *) "Sistemas de propulsão e controle autônomo, ativados!");
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "stop_immediately") == 0)
				{
					printf("Command detected: %s \n\n", "Parar imeditamente!");

					carmen_navigator_ackerman_stop();

					carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
					system("mpg123 $CARMEN_HOME/data/voice_interface_data/computerbeep_1.mp3"); // http://www.trekcore.com/audio/
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "stop") == 0)
				{
					printf("Command detected: %s \n\n", "Parar!");

					publish_voice_interface_command_message("0.0", SET_SPEED);

					carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
					system("mpg123 $CARMEN_HOME/data/voice_interface_data/computerbeep_1.mp3"); // http://www.trekcore.com/audio/
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "set_course") == 0)
				{
					printf("Command detected: %s \n\n", "Estabelecer curso");
					bool place_detected = false;
					for (unsigned int i = 0; i < rasa_server_response["entities"].size(); i++)
					{
						if (strcmp(rasa_server_response["entities"][i]["entity"].asString().c_str(), "place") == 0)
						{
							char *rddf_to_place = check_if_place_is_known(rasa_server_response["entities"][i]["value"].asString().c_str());
							if (rddf_to_place)
							{
								publish_voice_interface_command_message(rddf_to_place, SET_COURSE);

								speek_sentence((char *) ("Curso para " +
										rasa_server_response["entities"][i]["value"].asString() + " estabelecido.").c_str());

								place_detected = true;
							}
						}
					}
					if (!place_detected)
					{
						speek_sentence((char *) "Desculpe... Não identifiquei nenhum lugar de meu conhecimento em seu comando...");
						speek_sentence((char *) ("Você disse " + rasa_server_response["text"].asString() + "?").c_str());
						speek_sentence((char *) ("Se o que você disse está correto, favor incluir o lugar mencionado em minha base de dados."));
					}
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "greet") == 0)
				{
					printf("Command detected: %s \n\n", "Olá");
					speek_sentence((char *) "Olá!... Bom ter você aqui!");
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "thankyou") == 0)
				{
					printf("Command detected: %s \n\n", "Obrigada");
					speek_sentence((char *) "Por nada!");
				}
				else if (strcmp(rasa_server_response["intent"]["name"].asString().c_str(), "introduce_yourself") == 0)
				{
					printf("Command detected: %s \n\n", "Apresente-se");
					speek_sentence((char *) "Olá! Eu sou a Iara, o automóvel robótico autônomo inteligente da Ufes.");
					speek_sentence((char *) "Meu mecanismo de autonomia é baseado em localização precisa em mapas.");
					speek_sentence((char *) "Assim, não dependo de GPS.");
					speek_sentence((char *) "Os mapas que uso são construídos por mim mesma, com a ajuda de 32 raios laser.");
					speek_sentence((char *) "Possuo, também, câmeras cujas imagens são analisadas por redes neurais profundas.");
					speek_sentence((char *) "Com elas consigo ver pedestres, semáforos, faixas e outras sinalizações de trânsito relevantes.");
					speek_sentence((char *) "Estou às suas ordens!");
				}
				else
				{
					speek_sentence((char *) "Desculpe... Sua inteção parace clara, mas não sei o que fazer...");
					speek_sentence((char *) ("Você disse " + rasa_server_response["text"].asString() + "?").c_str());
					speek_sentence((char *) "Se foi isto, não sei como proceder.");
				}
			}
			else
			{
				speek_sentence((char *) "Desculpe... Não entendi claramente sua intenção...");
				speek_sentence((char *) ("Você disse " + rasa_server_response["text"].asString() + "?").c_str());
				speek_sentence((char *) "Se foi isto, não sei como proceder.");
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
	carmen_voice_interface_define_command_message();
}


void
carmen_voice_interface_initialize(/*char *language_code*/)
{
	setlocale(LC_ALL, "pt_BR.UTF-8");

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

	/*
	if (strcmp (argv[1], "en") == 0){
		printf("Awaiting hotword... \n\n");
	}else{
		if (strcmp (argv[1], "pt") == 0){
			printf("Aguardando hotword.... \n\n ");
		}else{
			return(printf("Use: ./voice_interface en or pt \n"));

		}
	}
	*/

	while (true)
	{
		int hotword_detection_result = hotword_detection();
		if (hotword_detection_result == 1) // hotword detected
		{
			snd_pcm_drop(capture_handle);

			printf("Hotword detected! \n\n");

			carmen_ipc_sleep(0.1); // Necessario para reconectar com o audio para tocar o som abaixo.
			system("mpg123 $CARMEN_HOME/data/voice_interface_data/computerbeep_4.mp3"); // http://www.trekcore.com/audio/

			printf("Awaiting for command: \n\n");

			
			char *voice_command = carmen_voice_interface_listen();
			execute_voice_command(voice_command);

			snd_pcm_prepare(capture_handle);
			snd_pcm_start(capture_handle);

			printf("Awaiting hotword... \n\n");
		}
		else if (hotword_detection_result == 2) // error
			printf ("Error in hotword detection\n");

		carmen_ipc_sleep(0.0);
	}

	return (0);
}
