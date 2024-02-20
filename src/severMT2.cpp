#include "serverMT2.h"

int iResult_MT2;

int ListenSocketMT2 , ClientSocketMT2;

struct sockaddr_in server_addressMT2;
struct sockaddr_in client_addressMT2;

char recvbuf_MT2[DEFAULT_BUFLEN_MT2] = "";
int recvbuflen_MT2 = DEFAULT_BUFLEN_MT2;

char sendbuf_MT2[DEFAULT_BUFLEN_MT2];


bool clientActiveMT2 = false;

void setupServerMT2() {

   ListenSocketMT2 = socket(AF_INET, SOCK_STREAM, 0);
   if(ListenSocketMT2 < 0)
   {

        std::cout<< "Error: socket creation failed for Monitoring Tool 2 server!" << std::endl;
        exit(0);

   }

    memset(&server_addressMT2, 0, sizeof(server_addressMT2));
    server_addressMT2.sin_family = AF_INET;
    server_addressMT2.sin_port = htons(DEFAULT_PORT_MT2);
    inet_pton(AF_INET, IP_ADDRESS_MT2, &server_addressMT2.sin_addr);


    if(bind(ListenSocketMT2, (struct sockaddr*)&server_addressMT2, sizeof(server_addressMT2)) <0)
    {
        std::cout<< "Error: socket binding failed for Monitoring Tool 2 server!"<<std::endl;
        exit(0);

    }

    if(listen(ListenSocketMT2, SOMAXCONN)< 0)
    {
        std::cout<< "Error: Listening for Monitoring Tool 2 failed!"<<std::endl;
        exit(0);
    }

    std::cout<< "Info: Monitoring Tool 2 server is waiting for client....." <<std::endl;


    socklen_t client_len = sizeof(client_addressMT2);
    ClientSocketMT2 = accept(ListenSocketMT2, (struct sockaddr*) &client_addressMT2, &client_len);
    if(ClientSocketMT2 < 0)
    {
        std::cout<< "Error: connection accept failed to Monitoring Tool 2 server!"<< std::endl;
        exit(0);
    }

    std::cout<< "Info: Successfully connected to Monitoring Tool 2 client" <<std::endl;
    close(ListenSocketMT2);
    clientActiveMT2 = true;

    std::cout << std::endl;
}

std::string receiveMessageMT2() {
    if (clientActiveMT2)
    {
        memset(&recvbuf_MT2[0], 0, sizeof(recvbuf_MT2));
        iResult_MT2 = recv(ClientSocketMT2, recvbuf_MT2, recvbuflen_MT2, 0);

        if (iResult_MT2 > 0)
        {
            return(std::string(recvbuf_MT2));
        }
        else if (iResult_MT2 == 0)
        {
            std::cout << "Error: socket connection closed! Terminating..." << std::endl;
            clientActiveMT2 = false;
            //exit(0);
            return "connection_error";

        }

        else
        {
            std::cout << "Error: receive failed ! " <<  std::endl;
            close(ClientSocketMT2);
            std::cout << "Socket connection closed! Terminating..." << std::endl;
            clientActiveMT2 = false;
            return "connection_error";
            //exit(0);
        }
    }
    else
    {
        std::cout << "Connection to Monitoring Tool 2 client was not possible! Terminating..." << std::endl;
        return "connection_error";
        //exit(0);
    }
}

void sendMessageMT2(std::string input) {
    if (clientActiveMT2)
    {
        strcpy(sendbuf_MT2, input.c_str());
        iResult_MT2 = send(ClientSocketMT2, sendbuf_MT2, (int)strlen(sendbuf_MT2), 0);
        if (iResult_MT2 == -1)
        {
            std::cout << "Error: send failed !" << std::endl;
            close(ClientSocketMT2);
            std::cout << "Socket connection closed with Monitoring Tool 2 client!" << std::endl;
            clientActiveMT2 = false;
        }
    }
}

void shutdownMT2() {
    if (clientActiveMT2) {
        std::cout << "Info: terminating socket connection by user request..." << std::endl;
        iResult_MT2 = shutdown(ClientSocketMT2, SHUT_WR);
        clientActiveMT2 = false;
        if (iResult_MT2 == -1) {
            std::cout << "Error: shutdown failed for Monitoring Tool 2!" << std::endl;
            close(ClientSocketMT2);
        }
    }
}