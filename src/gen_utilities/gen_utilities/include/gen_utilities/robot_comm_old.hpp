#define _BSD_SOURCE
#include <sys/socket.h>
#include <ros/ros.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <iostream>
#define SA struct sockaddr 

namespace robot_comm
{
    struct sockaddr_in servaddr, cli; 

    class robot_comm
    {
    protected:
    	int sockfd, connfd; 
        std::string ip_address;
        int port;
        char buff[1024]; 
        
    public:
        robot_comm(std::string Ip_address, int Port);

        bool establish_comm();

        void send_data(std::string str);

        std::string receive_data();

        void close_comm();
    };  
}  
