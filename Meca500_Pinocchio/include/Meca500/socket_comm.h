#ifndef SOCKET_H
#define SOCKET_H

#include <iostream>

/*
using boost::asio::ip::tcp;

typedef struct{
    boost::asio::ip::tcp::socket socket_;
}TcpIp_Connection;

typedef struct {
    int id;
    std::string name;
    float score;
} Student;
*/

class Socket_connection{

public:
     
    Socket_connection();
    ~Socket_connection();

    // Connect a TCP socket
    // Return 0 for success, -1 for failure.
    int TcpIp_connect(const char* address);
    
    // Disconnect the connection 
    void TcpIp_disconnect();
    
    // Send data to the buffer
    // Return 0 for success, -1 for failure, or the size that was actually sent
    // if the entire data did not fit in the TCP buffer. In that case, you should
    // send the remaining.
    int TcpIp_send(const char* buffer);
    
    // Try to receive len bytes of data.
    // Return 0 for CLOSED CONNECTION, -1 for failure, or the nb of data received.
    int TcpIp_receive();
    
private:
    bool success_ = false;
    
};
    
#endif // SOCKET_H
