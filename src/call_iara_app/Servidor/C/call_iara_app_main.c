#include <carmen/carmen.h>
#include <carmen/voice_interface_interface.h>
#include <carmen/voice_interface_messages.h>
#include "call_iara_app_messages.h"
#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <stdlib.h>
#include <string.h>


void
publish_voice_app_command_message(const char *command, int command_id)
{
	carmen_voice_interface_command_message message;

	message.command_id = command_id;
	message.command = (char *) command;
	message.timestamp = carmen_get_time();
	message.host = carmen_get_host();

	carmen_voice_interface_publish_command_message(&message);
}


void
get_annotation_from_rddf(char *allrddf)
{
	FILE *stream;
	char *line = NULL;
	size_t len = 0;
	ssize_t read;
	bzero(allrddf,3000);
	char buffer[1024];
	bzero(buffer,1024);
	strcat(buffer,getenv("CARMEN_HOME"));
	strcat(buffer,"/data/rddf_annotation_log_20140418.txt\0");
	stream = fopen(buffer, "r");
	if (stream == NULL)
		printf("Arquivo RDDF não encontrado!");
	else{

		while ((read = getline(&line, &len, stream)) != -1)
		{
			if(strstr(line,"RDDF_PLACE")){

				if(line[strlen(line)-sizeof(char)] == '\n')
					line[strlen(line)-sizeof(char)] = '#';

				strcat(allrddf,line);
			}
		}
	}
	free(line);
	fclose(stream);
}


carmen_app_solicitation_message
publish_app_solicitation_message(char * buffer)
{
	carmen_app_solicitation_message message_received;
	char temp[MAXSIZE];
	bzero(temp,MAXSIZE);
	int cont = 0;
	int index = 0;
	for(int i = 0 ; buffer[i] != '\0' ; i++)
	{
		if(buffer[i] == ';'){
			++cont;
			switch(cont)
			{
			case 1:
				message_received.reqnumber = temp[0] - '0';
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			case 2:
				strcpy(message_received.origin,temp);
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			case 3:
				strcpy(message_received.destination,temp);
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			case 4:
				strcpy(message_received.ipclient,temp);
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			}
		}
		else{
			temp[index]=buffer[i];
			index++;
		}
	}
	printf("Número da requisição: %d\n",message_received.reqnumber);
	printf("Origem: %s\n",message_received.origin);
	printf("Destino: %s\n",message_received.destination);
	printf("IP do cliente: %s\n",message_received.ipclient);
	printf("\n");
	return message_received;
}


char *
choose_rddf_file(carmen_app_solicitation_message message)
{

	static char rddf_file_name[2048];
	char *condition = message.origin;
//	char buffer[MAXSIZE];
	bzero(rddf_file_name,2048);
	//strcat(rddf_file_name,getenv("CARMEN_HOME"));
	strcat(rddf_file_name,"data/rndf/");
	//criar arquivos RDDF partindo do lcad ate o destino:
			//lcad X estacionamento ambiental
			//lcad X teatro
			//lcad X lagoa
	//setar o nome do arquivo na variável rddf_file_name
//	strcpy(rddf_file_name, rddf);

	//strcat(condition,message.origin);
	strcat(condition,"-");
	strcat(condition,message.destination);

	if(strcmp(condition,"RDDF_PLACE_LCAD-RDDF_PLACE_ESCADARIA_TEATRO") == 0){
		strcat(rddf_file_name,"rddf_log_volta_da_ufes-201903025-lcad-teatro.txt");
		return rddf_file_name;
	}
	//Estacionamento Ambiental
	if(strcmp(condition,"RDDF_PLACE_LCAD-RDDF_PLACE_CANTINA_CT")  == 0){
		strcat(rddf_file_name,"rddf_log_volta_da_ufes-201903025-lcad-estacionamento-ambiental.txt");
		return rddf_file_name;
	}
	if(strcmp(condition,"RDDF_PLACE_LCAD-RDDF_PLACE_LAGO") == 0){
		strcat(rddf_file_name,"rddf_log_volta_da_ufes-201903025-lcad-lagoa.txt");
		return rddf_file_name;
	}
	//LCAD to LCAD
	if(strcmp(condition,"RDDF_PLACE_LCAD-RDDF_PLACE_ESTACIONAMENTO_CCJE") == 0){
		strcat(rddf_file_name,"rddf_log_volta_da_ufes-201903025.txt");
		return rddf_file_name;
	}

	//return (rddf_file_name);
	return "0";

}


void
initiate_server(char * rddf_annotation)
{
	int socket_desc , new_socket , c;
	struct sockaddr_in server , client;
	char *message;
	char buffer[MAXSIZE];
	carmen_app_solicitation_message solicitation_message;
	char *rddf_file;

	//Create socket
	socket_desc = socket(AF_INET , SOCK_STREAM , 0);
	if (socket_desc == -1)
	{
		printf("Could not create socket");
	}

	//Prepare the sockaddr_in structure
	server.sin_family = AF_INET;
	server.sin_addr.s_addr = INADDR_ANY;
	server.sin_port = htons( 8000 );

	//Bind
	if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
	{
		puts("bind failed");
	}
	puts("bind done");

	//Listen
	listen(socket_desc , 3);

	//Accept and incoming connection
	puts("Waiting for incoming connections...");
	c = sizeof(struct sockaddr_in);
	while (1){
		while( (new_socket = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c)) )
		{
			//puts("Connection accepted");
			bzero(buffer,MAXSIZE);
			read(new_socket, buffer, sizeof(buffer));
			switch(buffer[0])
			{
			case '3':
				printf("Requisição de rddf: %s\n",buffer);
				message = rddf_annotation;
				break;
			case '2':
				printf("Solicitação de serviço: %s\n",buffer);
				message = (char *) "Requisição solicitada";
				solicitation_message = publish_app_solicitation_message(buffer);
				rddf_file = (char *) choose_rddf_file(solicitation_message);
				printf("Testando arquivo rddf: %s\n\n",rddf_file);
				publish_voice_app_command_message(rddf_file, SET_COURSE);
				carmen_navigator_ackerman_go();
				break;
			case '1':
				printf("Cancelamento de serviço: %s\n",buffer);
				message = (char *) "Requisição cancelada";
				solicitation_message = publish_app_solicitation_message(buffer);
				carmen_navigator_ackerman_stop();
				break;
			default:
				printf("Condição impossível: %s\n",buffer);
				message = (char *) "Condição teóricamente impossível.";
			}
			write(new_socket , message , strlen(message));

			//Coloca na mensagem do carmen
			if (new_socket<0)
			{
				perror("accept failed");
			}
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                           //
// Handlers                                                                                  //
//                                                                                           //
///////////////////////////////////////////////////////////////////////////////////////////////

void
shutdown_module(int signo)
{
    if (signo == SIGINT) {
        carmen_ipc_disconnect();
        printf("Call Iara Server: Disconnected.\n");
        exit(0);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////////


int
main(int argc , char **argv)
{
	carmen_ipc_initialize(argc, argv);

	signal(SIGINT, shutdown_module);

	char rddf_annotation[3000];

	get_annotation_from_rddf(rddf_annotation);

	initiate_server(rddf_annotation);

	carmen_ipc_dispatch();

	return 0;
}
