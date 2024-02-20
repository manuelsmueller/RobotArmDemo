#pragma once
#ifndef SERVEROBS_H_
#define SERVEROBS_H_

#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <vector>
#include "GlobalVariables.h"

#define BUFLEN_OBS 512
#define PORT_OBS 60000
#define IP_ADDRESS_OBS "192.168.0.102" //your IPV4 address


int ListenSocketObs, ClientSocketObs;

struct sockaddr_in server_address;
struct sockaddr_in client_address;

char recvbuf_Obs[BUFLEN_OBS] = "";
int recvbuflen_Obs = BUFLEN_OBS;

char sendbuf_Obs[BUFLEN_OBS];

bool clientActiveObs = false;
GlobalVariables *gv=GlobalVariables::get_instance();

void setupServerObs() {

   ListenSocketObs = socket(AF_INET, SOCK_STREAM, 0);
   if(ListenSocketObs < 0)
   {

        std::cout<< "Error: socket creation failed for obstacle server!" << std::endl;
        exit(0);

   }

    memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(PORT_OBS);
    inet_pton(AF_INET, IP_ADDRESS_OBS, &server_address.sin_addr);


    if(bind(ListenSocketObs, (struct sockaddr*)&server_address, sizeof(server_address)) <0)
    {
        std::cout<< "Error: socket binding failed for obstacle server!"<<std::endl;
        exit(0);

    }

    if(listen(ListenSocketObs, SOMAXCONN)< 0)
    {
        std::cout<< "Error: Listening for obstacle client failed!"<<std::endl;
        exit(0);
    }

    std::cout<< "Info: Obstacle server is waiting for client....." <<std::endl;


    socklen_t client_len = sizeof(client_address);
    ClientSocketObs = accept(ListenSocketObs, (struct sockaddr*) &client_address, &client_len);
    if(ClientSocketObs < 0)
    {
        std::cout<< "Error: connection accept failed tp obstacle server!"<< std::endl;
        exit(0);
    }

    std::cout<< "Info: Successfully connected to Obstacle Client" <<std::endl;
    close(ListenSocketObs);
    clientActiveObs = true;
}

void receiveMessageObs() {
    if (clientActiveObs)
    {
        memset(&recvbuf_Obs[0], 0, sizeof(recvbuf_Obs));
        int bytesReceived = recv(ClientSocketObs, recvbuf_Obs, recvbuflen_Obs, 0);

        if (bytesReceived > 0)
        {
            gv ->stateMessageObs = std::string(recvbuf_Obs);
        }
        else if (bytesReceived == 0)
        {
            std::cout << "Info: The client disconnected" << std::endl;
            clientActiveObs = false;
            gv ->stateMessageObs = "connection_error";

        }
        else if (bytesReceived < 0)
        {
            std::cout << "Error: socket connection closed! Terminating..." << std::endl;
            gv ->stateMessageObs = "connection_error";
            exit(0);

        }
        else
        {
            std::cout << "Error: receive failed ! " <<  std::endl;
            close(ClientSocketObs);
            std::cout << "Socket connection closed! Terminating..." << std::endl;
            clientActiveObs = false;
            gv ->stateMessageObs = "connection_error";
            exit(0);
        }
    }
    else
    {
        std::cout << "Connection to Obstacle client was not possible! Terminating..." << std::endl;
        gv ->stateMessageObs = "connection_error";
        exit(0);
    }
}

void sendMessageObs(std::string input) {
    if (clientActiveObs)
    {
        strcpy(sendbuf_Obs, input.c_str());
        int resultObs = send(ClientSocketObs, sendbuf_Obs, (int)strlen(sendbuf_Obs), 0);
        if (resultObs == -1)
        {
            std::cout << "Error: send failed !" << std::endl;
            close(ClientSocketObs);
            std::cout << "Socket connection closed with obstacle client!" << std::endl;
            clientActiveObs = false;
        }
    }
}

void shutdownObs() {
    if (clientActiveObs) {
        std::cout << "Info: terminating socket connection by user request..." << std::endl;
        int resultObs = shutdown(ClientSocketObs, SHUT_WR);
        clientActiveObs = false;
        if (resultObs == -1) {
            std::cout << "Error: shutdown failed!" << std::endl;
            close(ClientSocketObs);
        }
    }
}

#endif
