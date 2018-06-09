#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#define PORT "8080"
#define TCP_IP_ADDRESS			"10.42.0.28"
  
int main(int argc, char const *argv[])
{
    struct addrinfo host_info;       // The struct that getaddrinfo() fills up with data.
    struct addrinfo *host_info_list; // Pointer to the to the linked list of host_info's.
    int sock = 0, valread, status;
    char *hello = "Hello from client";
    char buffer[1024] = {0};
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("\n Socket creation error \n");
        return -1;
    }
  
    memset(&host_info, 0, sizeof host_info);
      
	host_info.ai_family = AF_UNSPEC;     // IP version not specified. Can be both.
    host_info.ai_socktype = SOCK_STREAM; // Use SOCK_STREAM for TCP or SOCK_DGRAM for UDP.

    status = getaddrinfo(TCP_IP_ADDRESS, PORT, &host_info, &host_info_list);
    if (status != 0)
    {
        printf("Erro no Get_Addrinfo\n");
        return (-1);
    }

    status = connect(sock, host_info_list->ai_addr, host_info_list->ai_addrlen);
    if(status < 0){
        printf("\nConnection Failed \n");
        return (-1);
    }
    send(sock , hello , strlen(hello) , 0 );
    printf("Hello message sent\n");
    valread = read( sock , buffer, 1024);
    printf("%s\n",buffer );
    return 0;
}