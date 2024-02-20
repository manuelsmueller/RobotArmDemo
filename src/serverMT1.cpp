#include "serverMT1.h"

int iResult_MT1;

int ListenSocketMT1 , ClientSocketMT1;

struct sockaddr_in server_addressMT1;
struct sockaddr_in client_addressMT1;

char recvbuf_MT1[DEFAULT_BUFLEN_MT1] = "";
int recvbuflen_MT1 = DEFAULT_BUFLEN_MT1;

char sendbuf_MT1[DEFAULT_BUFLEN_MT1];


bool clientActiveMT1 = false;

void setupServerMT1() {

   ListenSocketMT1 = socket(AF_INET, SOCK_STREAM, 0);
   if(ListenSocketMT1 < 0)
   {

        std::cout<< "Error: socket creation failed for Monitoring Tool 1 server!" << std::endl;
        exit(0);

   }

    memset(&server_addressMT1, 0, sizeof(server_addressMT1));
    server_addressMT1.sin_family = AF_INET;
    server_addressMT1.sin_port = htons(DEFAULT_PORT_MT1);
    inet_pton(AF_INET, IP_ADDRESS_MT1, &server_addressMT1.sin_addr);


    if(bind(ListenSocketMT1, (struct sockaddr*)&server_addressMT1, sizeof(server_addressMT1)) <0)
    {
        std::cout<< "Error: socket binding failed for Monitoring Tool 1 server!"<<std::endl;
        exit(0);

    }

    if(listen(ListenSocketMT1, SOMAXCONN)< 0)
    {
        std::cout<< "Error: Listening for Monitoring Tool 1 failed!"<<std::endl;
        exit(0);
    }

    std::cout<< "Info: Monitoring Tool 1 server is waiting for client....." <<std::endl;


    socklen_t client_len = sizeof(client_addressMT1);
    ClientSocketMT1 = accept(ListenSocketMT1, (struct sockaddr*) &client_addressMT1, &client_len);
    if(ClientSocketMT1 < 0)
    {
        std::cout<< "Error: connection accept failed to Monitoring Tool 1 server!"<< std::endl;
        exit(0);
    }

    std::cout<< "Info: Successfully connected to Monitoring Tool 1 Client" <<std::endl;
    close(ListenSocketMT1);
    clientActiveMT1 = true;

    std::cout << std::endl;
}

std::string receiveMessageMT1() {
    if (clientActiveMT1)
    {
        memset(&recvbuf_MT1[0], 0, sizeof(recvbuf_MT1));
        iResult_MT1 = recv(ClientSocketMT1, recvbuf_MT1, recvbuflen_MT1, 0);

        if (iResult_MT1 > 0)
        {
            return(std::string(recvbuf_MT1));
        }
        else if (iResult_MT1 == 0)
        {
            std::cout << "Error: socket connection closed! Terminating..." << std::endl;
            clientActiveMT1 = false;
            //exit(0);
            return "connection_error";

        }

        else
        {
            std::cout << "Error: receive failed ! " <<  std::endl;
            close(ClientSocketMT1);
            std::cout << "Socket connection closed! Terminating..." << std::endl;
            clientActiveMT1 = false;
            return "connection_error";
            //exit(0);
        }
    }
    else
    {
        std::cout << "Connection to Monitoring Tool 1 client was not possible! Terminating..." << std::endl;
        return "connection_error";
        //exit(0);
    }
}

void sendMessageMT1(std::string input) {
    if (clientActiveMT1)
    {
        strcpy(sendbuf_MT1, input.c_str());
        iResult_MT1 = send(ClientSocketMT1, sendbuf_MT1, (int)strlen(sendbuf_MT1), 0);
        if (iResult_MT1 == -1)
        {
            std::cout << "Error: send failed !" << std::endl;
            close(ClientSocketMT1);
            std::cout << "Socket connection closed with Monitoring Tool 1 client!" << std::endl;
            clientActiveMT1 = false;
        }
    }
}

void shutdownMT1() {
    if (clientActiveMT1) {
        std::cout << "Info: terminating socket connection by user request..." << std::endl;
        iResult_MT1 = shutdown(ClientSocketMT1, SHUT_WR);
        clientActiveMT1 = false;
        if (iResult_MT1 == -1) {
            std::cout << "Error: shutdown failed for Monitoring Tool 1!" << std::endl;
            close(ClientSocketMT1);
        }
    }
}