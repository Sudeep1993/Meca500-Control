#include <iostream>
#include "Meca500/socket_comm.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

const int BUFFER_SIZE = 1024;
static int sock=0;
static int socket_system_user_count =0;

Socket_connection::Socket_connection()
{

}

Socket_connection::~Socket_connection()
{

}



/// Initialise the socket library if needed.
int init_socket_system()
{
    if (socket_system_user_count == 0)
    {
        printf("Init Scocket\r\n");
    	sock = socket(AF_INET, SOCK_STREAM, 0);

    	if (sock == -1) {
        	std::cerr << "Failed to create socket" << std::endl;
        	return -1;
    	}
        
       
        // Success!
        std::cout << "Socket system initialized successfully." << std::endl;
    }
    socket_system_user_count++;
    return 0;
}


/// Close the socket library if no more user on it.
void cleanup_socket_system() {
    socket_system_user_count--;
    if (socket_system_user_count==0)
    {
        std::cout << "Closing socket system." << std::endl;
        // Create an io_context object and run it with the stop() method to clean up
    	close(sock);
    	std::cout << "socket system Closed" << std::endl;
    }
}

void receive_data()
{
	char buffer[BUFFER_SIZE];
    ssize_t bytes_received = recv(sock, buffer, BUFFER_SIZE, 0);
    if (bytes_received == -1) {
        std::cerr << "Failed to receive data" << std::endl;
        return;
    }
    std::string response(buffer, bytes_received);
    std::cout << "Response: " << response << std::endl;
    return;
    
}

//=============================================================================
//
int Socket_connection::TcpIp_connect(const char* address){
	int result = 0;
	result = init_socket_system();
	if (result != 0)
	{
		return -1;
	}
    
    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = inet_addr(address);
    server.sin_port = htons(10000);
    
    if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
    	cleanup_socket_system();
        std::cerr << "Failed to connect to server" << std::endl;
        
        return -1;
    }
    receive_data();
    return 0;
}
void Socket_connection::TcpIp_disconnect(){
	if (socket_system_user_count == 0)
	{
		return;
	}
	cleanup_socket_system();
	
}


int Socket_connection::TcpIp_send(const char* buffer){
	if (sock == -1)
	{
		cleanup_socket_system();
		std::cerr << "Inmature Socket Closed" << std::endl;
		return -1;
	}
	
	ssize_t bytes_sent = send(sock, buffer, strlen(buffer), 0);
    if (bytes_sent == -1) {
        std::cerr << "Failed to send data" << std::endl;
    	cleanup_socket_system();
        return -1;
    }
    receive_data();
	return 0;
}
    

int Socket_connection::TcpIp_receive(){
	receive_data();
	return 0;
}


