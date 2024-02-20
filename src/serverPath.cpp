#include "serverPath.h"

int iResult_PATH;

int ListenSocketPATH , ClientSocketPATH;

struct sockaddr_in server_addressPATH;
struct sockaddr_in client_addressPATH;

char recvbuf_PATH[DEFAULT_BUFLEN_PATH] = "";
int recvbuflen_PATH = DEFAULT_BUFLEN_PATH;

char sendbuf_PATH[DEFAULT_BUFLEN_PATH];


bool clientActivePATH = false;

void setupServerPATH(bool virtualMode) {

   ListenSocketPATH = socket(AF_INET, SOCK_STREAM, 0);
   if(ListenSocketPATH < 0)
   {

        std::cout<< "Error: socket creation failed for Kuka path server!" << std::endl;
        exit(0);

   }

    memset(&server_addressPATH, 0, sizeof(server_addressPATH));
    server_addressPATH.sin_family = AF_INET;
    server_addressPATH.sin_port = htons(DEFAULT_PORT_PATH);
    if (virtualMode) inet_pton(AF_INET, "192.168.0.102", &server_addressPATH.sin_addr);
    else inet_pton(AF_INET, IP_ADDRESS_PATH, &server_addressPATH.sin_addr);


    if(bind(ListenSocketPATH, (struct sockaddr*)&server_addressPATH, sizeof(server_addressPATH)) <0)
    {
        std::cout<< "Error: socket binding failed for Kuka Path server!"<<std::endl;
        exit(0);

    }

    if(listen(ListenSocketPATH, SOMAXCONN)< 0)
    {
        std::cout<< "Error: Listening for Kuka Path failed!"<<std::endl;
        exit(0);
    }

    std::cout<< "Info: Kuka Path server is waiting for client....." <<std::endl;


    socklen_t client_len = sizeof(client_addressPATH);
    ClientSocketPATH = accept(ListenSocketPATH, (struct sockaddr*) &client_addressPATH, &client_len);
    if(ClientSocketPATH < 0)
    {
        std::cout<< "Error: connection accept failed to Kuka Path server!"<< std::endl;
        exit(0);
    }

    std::cout<< "Info: Successfully connected to Kuka Path client" <<std::endl;
    close(ListenSocketPATH);
    clientActivePATH = true;

    std::cout << std::endl;
}

std::string receiveMessagePATH() {
    if (clientActivePATH)
    {
        memset(&recvbuf_PATH[0], 0, sizeof(recvbuf_PATH));
        iResult_PATH = recv(ClientSocketPATH, recvbuf_PATH, recvbuflen_PATH, 0);

        if (iResult_PATH > 0)
        {
            return(std::string(recvbuf_PATH));
        }
        else if (iResult_PATH == 0)
        {
            std::cout << "Error: socket connection closed! Terminating..." << std::endl;
            clientActivePATH = false;
            //exit(0);
            return "connection_error";

        }

        else
        {
            std::cout << "Error: receive failed ! " <<  std::endl;
            close(ClientSocketPATH);
            std::cout << "Socket connection closed! Terminating..." << std::endl;
            clientActivePATH = false;
            return "connection_error";
            //exit(0);
        }
    }
    else
    {
        std::cout << "Connection to Kuka Path client was not possible! Terminating..." << std::endl;
        return "connection_error";
        //exit(0);
    }
}

void sendMessagePATH(std::string input) {
    if (clientActivePATH)
    {
        strcpy(sendbuf_PATH, input.c_str());
        iResult_PATH = send(ClientSocketPATH, sendbuf_PATH, (int)strlen(sendbuf_PATH), 0);
        if (iResult_PATH == -1)
        {
            std::cout << "Error: send failed !" << std::endl;
            close(ClientSocketPATH);
            std::cout << "Socket connection closed with Kuka Path client!" << std::endl;
            clientActivePATH = false;
        }
    }
}

void shutdownPATH() {
    if (clientActivePATH) {
        std::cout << "Info: terminating socket connection by user request..." << std::endl;
        iResult_PATH = shutdown(ClientSocketPATH, SHUT_WR);
        clientActivePATH = false;
        if (iResult_PATH == -1) {
            std::cout << "Error: shutdown failed for Kuka Path!" << std::endl;
            close(ClientSocketPATH);
        }
    }
}
