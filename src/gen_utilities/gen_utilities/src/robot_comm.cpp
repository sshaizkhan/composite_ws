#define _BSD_SOURCE
#include <sys/socket.h>
#include "gen_utilities/robot_comm.hpp"
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <string>
#include <iostream>
#include <stdint.h>
#include <vector>
#include <Eigen/Eigen>

namespace robot_comm
{
    struct sockaddr_in servaddr, cliaddr; 
    RobotComm::RobotComm(std::string Ip_address, int Port, std::string socket_type)
	{
        ip_address = Ip_address;
        port = Port;
        Socket_Type = socket_type;
    }

    RobotComm::~RobotComm()
    {
        this->closeComm();
    }

    bool RobotComm::establishComm()
    {
        if (Socket_Type.compare("client")==0)
        {
            sockfd = socket(AF_INET, SOCK_STREAM, 0); 
            if (sockfd == -1) 
            { 
                std::cout << "Client socket creation failed..." << std::endl;
                exit(0); 
            } 
            else
            {
                std::cout << "Client Socket successfully created...!" << std::endl; 
            }
            bzero(&servaddr, sizeof(servaddr));   
            servaddr.sin_family = AF_INET; 
            servaddr.sin_addr.s_addr = inet_addr(ip_address.c_str()); 
            servaddr.sin_port = htons(port); 
            connect(sockfd, (SA*)&servaddr, sizeof(servaddr));
            std::string str = "Establish Communication\r\n";
            write(sockfd, str.c_str(), sizeof(str)); 
            bzero(buff, sizeof(buff)); 
            read(sockfd, buff, sizeof(buff)); 
            std::string data(buff);
            if (data.find("Okay")==0)
            {
                std::cout << "Communication established to the robot!" << std::endl;
                primesockfd = sockfd;
                return true;
            }
            else
            {
                std::cout << "Communication could not be established..." << std::endl;
                primesockfd = sockfd;
                return false;
            }
        }
        if (Socket_Type.compare("server")==0)
        {
            sockfd = socket(AF_INET, SOCK_STREAM, 0); 
            int option = 1;
            setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &option, sizeof(option));
            if (sockfd < 0) 
            { 
                std::cout << "Server socket creation failed..." << std::endl;
                exit(1); 
            } 
            else
            {
                std::cout << "Server Socket successfully created...!" << std::endl; 
            }
            bzero(&servaddr, sizeof(servaddr));   
            servaddr.sin_family = AF_INET; 
            servaddr.sin_addr.s_addr = INADDR_ANY;
            servaddr.sin_port = htons(port); 
            if (bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) < 0) 
            {
                perror("Binding ERROR");
                exit(1);
            }
            listen(sockfd,5);
            clilen = sizeof(cliaddr);
            newsockfd = accept(sockfd, (struct sockaddr *) &cliaddr, (socklen_t*) &clilen);
            if (newsockfd < 0) 
            {
                perror("Accept ERROR");
                exit(1);
            }
            bzero(buff, sizeof(buff)); 
            read(newsockfd, buff, sizeof(buff));
            std::string data(buff);
            data = data.substr(0,data.size()-2);
            if (data.compare("Establish_Communication")==0)
            {
                std::string str = "Okay\r\n";
                write(newsockfd, str.c_str(), sizeof(str)); 
                std::cout << "Communication established to the robot!" << std::endl;
                primesockfd = newsockfd;
                return true;
            }
            else
            {
                std::cout << "Communication could not be established to the robot!" << std::endl;
                primesockfd = newsockfd;
                return false;
            }
        }
    };

/////////////////////////////////// send methods ///////////////////////////////////////////
    
    void RobotComm::sendString(std::string str)
    {
        std::string out_data = str + "\r\n";
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char), 0); 
    };

    void RobotComm::sendInt(int data1)
    {
        std::string out_data = std::to_string(data1) + "\r\n";
        send(primesockfd, out_data.c_str(), sizeof(out_data), 0); 
	};

    void RobotComm::sendDouble(double data1)
    {
        std::string out_data = std::to_string(data1) + "\r\n";
        send(primesockfd, out_data.c_str(), sizeof(out_data), 0); 
    };

    void RobotComm::sendIntArray(std::vector<int> data)
    {
        std::string out_data = "";
        for (int i=0;i<data.size();++i)
        {
            if (i==data.size()-1)
            {
                out_data += std::to_string(data[i]) + "\r\n";
                continue;
            } 
            out_data += std::to_string(data[i]) + ",";
        }
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char) ,0);
    };

    void RobotComm::sendIntArray(Eigen::MatrixXi data)
    {
        std::string out_data = "";
        for (int i=0;i<data.cols();++i)
        {
            if (i==data.cols()-1)
            {
                out_data += std::to_string(data(0,i)) + "\r\n";
                continue;
            } 
            out_data += std::to_string(data(0,i)) + ",";
        }
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char) ,0);
    };


    void RobotComm::sendFloatArray(std::vector<float> data)
    {
        std::string out_data = "";
        for (int i=0;i<data.size();++i)
        {
            if (i==data.size()-1)
            {
                out_data += std::to_string(data[i]) + "\r\n";
                continue;
            } 
            out_data += std::to_string(data[i]) + ",";
        }
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char) ,0);
    };

    void RobotComm::sendFloatArray(Eigen::MatrixXf data)
    {
        std::string out_data = "";
        for (int i=0;i<data.cols();++i)
        {
            if (i==data.cols()-1)
            {
                out_data += std::to_string(data(0,i)) + "\r\n";
                continue;
            } 
            out_data += std::to_string(data(0,i)) + ",";
        }
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char) ,0);
    };

    void RobotComm::sendDoubleArray(std::vector<double> data)
    {
        std::string out_data = "";
        for (int i=0;i<data.size();++i)
        {
            if (i==data.size()-1)
            {
                out_data += std::to_string(data[i]) + "\r\n";
                continue;
            } 
            out_data += std::to_string(data[i]) + ",";
        }
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char) ,0);
    };

    void RobotComm::sendDoubleArray(Eigen::MatrixXd data)
    {
        std::string out_data = "";
        for (int i=0;i<data.cols();++i)
        {
            if (i==data.cols()-1)
            {
                out_data += std::to_string(data(0,i)) + "\r\n";
                continue;
            } 
            out_data += std::to_string(data(0,i)) + ",";
        }
        send(primesockfd, out_data.c_str(), out_data.length()*sizeof(char) ,0);
    };

    void RobotComm::sendIntMatrix(std::vector<std::vector<int>> data)
    {
        sendInt(data.size());
        receiveString();
        for (int i=0;i<data.size();++i)
        {
            sendIntArray(data[i]);
            receiveString();    
        }
    };

    void RobotComm::sendIntMatrix(Eigen::MatrixXi data)
    {
        sendInt(data.rows());
        receiveString();
        for (int i=0;i<data.rows();++i)
        {
            sendIntArray(data.row(i));
            receiveString();    
        }
    };

    void RobotComm::sendFloatMatrix(std::vector<std::vector<float>> data)
    {
        sendInt(data.size());
        receiveString();
        for (int i=0;i<data.size();++i)
        {
            sendFloatArray(data[i]);
            receiveString();    
        }
    };

    void RobotComm::sendFloatMatrix(Eigen::MatrixXf data)
    {
        sendInt(data.rows());
        receiveString();
        for (int i=0;i<data.rows();++i)
        {
            sendFloatArray(data.row(i));
            receiveString();    
        }
    };

    void RobotComm::sendDoubleMatrix(std::vector<std::vector<double>> data)
    {
        sendInt(data.size());
        receiveString();
        for (int i=0;i<data.size();++i)
        {
            sendDoubleArray(data[i]);
            receiveString();    
        }
    };

    void RobotComm::sendDoubleMatrix(Eigen::MatrixXd data)
    {
        sendInt(data.rows());
        receiveString();
        for (int i=0;i<data.rows();++i)
        {
            sendDoubleArray(data.row(i));
            receiveString();    
        }
    };

/////////////////////////////////// receive methods ///////////////////////////////////////////

    std::string RobotComm::receiveString()
    {
        bzero(buff, sizeof(buff)); 
        read(primesockfd, buff, sizeof(buff));
        std::string in_data(buff);    
        return in_data.substr(0,in_data.size()-2);
    };

    int RobotComm::receiveInt()
    {
        bzero(buff, sizeof(buff)); 
        read(primesockfd, buff, sizeof(buff));
        std::string in_data(buff);    
        return std::stoi(in_data.substr(0,in_data.size()-2));
    };

	double RobotComm::receiveDouble()
    {
        bzero(buff, sizeof(buff)); 
        read(primesockfd, buff, sizeof(buff));
        std::string in_data(buff);    
        return std::stod(in_data.substr(0,in_data.size()-2));
    };

    std::vector <int> RobotComm::receiveInt1DVec()
    {
        int size = receiveInt();
        sendString("y");
        std::vector<int> arr;
        for (int i=0;i<size;++i)
        {
            arr.push_back(receiveInt());
            sendString("y");
        }
        return arr;
    };

    Eigen::MatrixXi RobotComm::receiveInt1DMat()
    {
        int size = receiveInt();
        sendString("y");
        Eigen::MatrixXi arr(1,size);
        for (int i=0;i<size;++i)
        {
            arr(0,i) = receiveInt();
            sendString("y");
        }
        return arr;
    };

    std::vector <double> RobotComm::receiveDouble1DVec()
    {
        int size = receiveInt();
        sendString("y");
        std::vector<double> arr;
        for (int i=0;i<size;++i)
        {
            arr.push_back(receiveDouble());
            sendString("y");
        }
        return arr;
    };

    Eigen::MatrixXd RobotComm::receiveDouble1DMat()
    {
        int size = receiveInt();
        sendString("y");
        Eigen::MatrixXd arr(1,size);
        for (int i=0;i<size;++i)
        {
            arr(0,i) = receiveDouble();
            sendString("y");
        }
        return arr;
    };

    std::vector<std::vector <int>> RobotComm::receiveIntVec()
    {
        int rows = receiveInt();
        sendString("y");
        int cols = receiveInt();
        sendString("y");
        std::vector<std::vector <int> > mat;
        for (int i=0;i<rows;++i)
        {
            mat.push_back(receiveInt1DVec());
            // sendString("y");
        }
        return mat;
    };

    Eigen::MatrixXi RobotComm::receiveIntMat()
    {
        int rows = receiveInt();
        sendString("y");
        int cols = receiveInt();
        sendString("y");
        Eigen::MatrixXi mat(rows,cols);
        for (int i=0;i<rows;++i)
        {
            mat.row(i) = receiveInt1DMat();
            // sendString("y");
        }
        return mat;
    };

    std::vector<std::vector <double>> RobotComm::receiveDoubleVec()
    {
        int rows = receiveInt();
        sendString("y");
        int cols = receiveInt();
        sendString("y");
        std::vector<std::vector <double> > mat;
        for (int i=0;i<rows;++i)
        {
            mat.push_back(receiveDouble1DVec());
            // sendString("y");
        }
        return mat;
    };

    Eigen::MatrixXd RobotComm::receiveDoubleMat()
    {
        int rows = receiveInt();
        sendString("y");
        int cols = receiveInt();
        sendString("y");
        std::cout << "rows : " << rows <<  " , cols : " << cols << std::endl;
        Eigen::MatrixXd mat(rows,cols);
        for (int i=0;i<rows;++i)
        {
            mat.row(i) = receiveDouble1DMat();
            // std::cout << mat.row(i) << std::endl;
            // sendString("y");
        }
        return mat;
    };

/////////////////////////////////// receive methods ///////////////////////////////////////////

    void RobotComm::closeComm()
    {
        if (Socket_Type.compare("client")==0)
        {
            std::cout << "closing socket for client..." << std::endl;
            close(sockfd); 
            close(primesockfd); 
        }
        if (Socket_Type.compare("server")==0)
        {
            std::cout << "closing socket for server..." << std::endl;
            close(sockfd);     
            close(primesockfd);
        }
    };

};