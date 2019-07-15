#include<stdio.h>
#include<string.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<unistd.h>
#include <stdlib.h>
#include <string.h>
#include "call_iara_app_messages.h"

void
retorna(char *allrddf)
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
				//			line[ strlen(line)-1 ] ='#';
				if(line[strlen(line)-sizeof(char)] == '\n'){
					//  			printf("%c\n",line[strlen(line)-sizeof(char)]);
					line[strlen(line)-sizeof(char)] = '#';
				}
				strcat(allrddf,line);
			}
		}
	}
	//	printf("%s\n",allrddf);
	free(line);
	fclose(stream);
}

void
montamensagem(char * buffer)
{
	carmen_app_solicitation_message mensagem;
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
				mensagem.reqnumber = temp[0] - '0';
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			case 2:
				strcpy(mensagem.origem,temp);
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			case 3:
				strcpy(mensagem.destino,temp);
				bzero(temp,MAXSIZE);
				index = 0;
				break;
			case 4:
				strcpy(mensagem.ipcliente,temp);
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
	printf("Número da requisição: %d\n",mensagem.reqnumber);
	printf("Origem: %s\n",mensagem.origem);
	printf("Destino: %s\n",mensagem.destino);
	printf("IP do cliente: %s\n",mensagem.ipcliente);
	printf("\n");

}


int
main(int argc , char *argv[])
{
	int socket_desc , new_socket , c, r;
	struct sockaddr_in server , client;
	char *message;
	char buffer[MAXSIZE];
	char rddf[3000];
	int cont;
	retorna(rddf);

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
		return 1;
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
			r = read(new_socket, buffer, sizeof(buffer));
			switch(buffer[0])
			{
			case '3':
				printf("Requisição de rddf: %s\n",buffer);
				message = rddf;
				break;
			case '2':
				printf("Solicitação de serviço: %s\n",buffer);
				message = "Requisição solicitada";
				montamensagem(buffer);
				break;
			case '1':
				printf("Cancelamento de serviço: %s\n",buffer);
				message = "Requisição cancelada";
				montamensagem(buffer);
				break;
			default:
				printf("Condição impossível: %s\n",buffer);
				message = "Condição teóricamente impossível.";
			}
			write(new_socket , message , strlen(message));

			//Coloca na mensagem do carmen
			if (new_socket<0)
			{
				perror("accept failed");
				return 1;
			}
		}
	}
	return 0;
}
