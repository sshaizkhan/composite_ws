#define _BSD_SOURCE
#include <sys/socket.h>
#include <ros/ros.h>
#include "gen_utilities/robot_comm_old.hpp"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <iostream>
#define SA struct sockaddr 
  
struct sockaddr_in servaddr, cli; 

namespace robot_comm
{
    robot_comm::robot_comm(std::string Ip_address, int Port)
	{
        ip_address = Ip_address;
        port = Port;
    }

    bool robot_comm::establish_comm()
    {
        sockfd = socket(AF_INET, SOCK_STREAM, 0); 
        if (sockfd == -1) 
        { 
            std::cout << "socket creation failed..." << std::endl;
            exit(0); 
        } 
        else
        {
            std::cout << "Socket successfully created...!" << std::endl; 
        }
        bzero(&servaddr, sizeof(servaddr));   
        servaddr.sin_family = AF_INET; 
        servaddr.sin_addr.s_addr = inet_addr(ip_address.c_str()); 
        servaddr.sin_port = htons(port); 
        connect(sockfd, (SA*)&servaddr, sizeof(servaddr));
        std::string str = "Establish Communication\r\n";
        ssize_t statw = write(sockfd, str.c_str(), sizeof(str)); 
        bzero(buff, sizeof(buff)); 
        ssize_t statr = read(sockfd, buff, sizeof(buff)); 
        std::string data(buff);
        if (data.find("Okay")==0)
        {
            std::cout << "Communication established to the robot!" << std::endl;
            return true;
        }
        else
        {
            std::cout << "Communication could not be established..." << std::endl;
            return false;
        }
    };

    void robot_comm::send_data(std::string str)
    {
        std::string out_data = str + "\r\n";
        ssize_t statw = write(sockfd, out_data.c_str(), sizeof(out_data)); 
    };

    std::string robot_comm::receive_data()
    {
        bzero(buff, sizeof(buff)); 
        ssize_t statr = read(sockfd, buff, sizeof(buff));
        std::string in_data(buff);    
        return in_data.substr(0,in_data.size()-2);
        ;
    };

    void robot_comm::close_comm()
    {
        close(sockfd); 
    };

};