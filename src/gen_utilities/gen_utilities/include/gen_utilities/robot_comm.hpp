#define _BSD_SOURCE
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h>
#include <string> 
#include <iostream>
#include <vector>
#include <Eigen/Eigen>
#define SA struct sockaddr 

// struct sockaddr_in servaddr, cli; 

namespace robot_comm
{
    class RobotComm
    {
    protected:
    	int sockfd, connfd, newsockfd, clilen, primesockfd; 
        std::string ip_address, Socket_Type;
        int port;
        char buff[1024]; 
        
    public:
        //! establish TCPIP socket with port, ip and the socket type (server or client)
        RobotComm(std::string Ip_address, int Port, std::string socket_type="client");
        ~RobotComm();

        //! establish communication with other socket
        bool establishComm();

        //! data sending methods
        //! send a string over TCP
        void sendString(std::string);
        //! send an integer over TCP
        void sendInt(int);
        //! send a double over TCP
        void sendDouble(double);
        //! send an integer array over TCP
        void sendIntArray(std::vector<int> );
        void sendIntArray(Eigen::MatrixXi );
        //! send a float array over TCP
        void sendFloatArray(std::vector<float> );
        void sendFloatArray(Eigen::MatrixXf );
        //! send a double array over TCP
        void sendDoubleArray(std::vector<double> );
        void sendDoubleArray(Eigen::MatrixXd );
        //! send an integer matrix (2-D array) over TCP
        void sendIntMatrix(Eigen::MatrixXi);
        void sendIntMatrix(std::vector<std::vector<int>>);
        //! send a float matrix (2-D array) over TCP
        void sendFloatMatrix(Eigen::MatrixXf);
        void sendFloatMatrix(std::vector<std::vector<float>>);
        //! send a double matrix (2-D array) over TCP
        void sendDoubleMatrix(Eigen::MatrixXd);
        void sendDoubleMatrix(std::vector<std::vector<double>>);

        //! data receiving methods
        //! receive a string over TCP
        std::string receiveString();
        //! receive an integer over TCP
        int receiveInt();
        //! receive a double over TCP
        double receiveDouble();
        //! receive an integer array over TCP
        std::vector <int> receiveInt1DVec();
        Eigen::MatrixXi receiveInt1DMat();
        //! receive an double array over TCP
        std::vector <double> receiveDouble1DVec();
        Eigen::MatrixXd receiveDouble1DMat();
        //! receive an integer matrix (2-D array) over TCP
        std::vector<std::vector <int>> receiveIntVec();
        Eigen::MatrixXi receiveIntMat();
        //! receive a double matrix (2-D array) over TCP
        std::vector<std::vector <double>> receiveDoubleVec();
        Eigen::MatrixXd receiveDoubleMat();

        //! close socket 
        void closeComm();

    };  
}  
