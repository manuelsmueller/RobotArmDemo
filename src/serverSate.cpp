#include "serverState.h"

int iResult_STATE;

int ListenSocketSTATE , ClientSocketSTATE;

struct sockaddr_in server_addressSTATE;
struct sockaddr_in client_addressSTATE;

char recvbuf_STATE[DEFAULT_BUFLEN_STATE] = "";
int recvbuflen_STATE = DEFAULT_BUFLEN_STATE;

char sendbuf_STATE[DEFAULT_BUFLEN_STATE];


bool clientActiveSTATE = false;

void setupServerSTATE(bool virtualMode) {

   ListenSocketSTATE = socket(AF_INET, SOCK_STREAM, 0);
   if(ListenSocketSTATE < 0)
   {

        std::cout<< "Error: socket creation failed for Kuka States erver!" << std::endl;
        exit(0);

   }

    memset(&server_addressSTATE, 0, sizeof(server_addressSTATE));
    server_addressSTATE.sin_family = AF_INET;
    server_addressSTATE.sin_port = htons(DEFAULT_PORT_STATE);
    if (virtualMode) inet_pton(AF_INET, "192.168.0.102", &server_addressSTATE.sin_addr);
    else inet_pton(AF_INET, IP_ADDRESS_STATE, &server_addressSTATE.sin_addr);


    if(bind(ListenSocketSTATE, (struct sockaddr*)&server_addressSTATE, sizeof(server_addressSTATE)) <0)
    {
        std::cout<< "Error: socket binding failed for Kuka State server!"<<std::endl;
        exit(0);

    }

    if(listen(ListenSocketSTATE, SOMAXCONN)< 0)
    {
        std::cout<< "Error: Listening for Kuka State failed!"<<std::endl;
        exit(0);
    }

    std::cout<< "Info: Kuka State server is waiting for client....." <<std::endl;


    socklen_t client_len = sizeof(client_addressSTATE);
    ClientSocketSTATE = accept(ListenSocketSTATE, (struct sockaddr*) &client_addressSTATE, &client_len);
    if(ClientSocketSTATE < 0)
    {
        std::cout<< "Error: connection accept failed to Kuka State server!"<< std::endl;
        exit(0);
    }

    std::cout<< "Info: Successfully connected to Kuka State client" <<std::endl;
    close(ListenSocketSTATE);
    clientActiveSTATE = true;

    std::cout << std::endl;
}

std::string receiveMessageSTATE() {
    if (clientActiveSTATE)
    {
        memset(&recvbuf_STATE[0], 0, sizeof(recvbuf_STATE));
        iResult_STATE = recv(ClientSocketSTATE, recvbuf_STATE, recvbuflen_STATE, 0);

        if (iResult_STATE > 0)
        {
            return(std::string(recvbuf_STATE));
        }
        else if (iResult_STATE == 0)
        {
            std::cout << "Error: socket connection closed! Terminating..." << std::endl;
            clientActiveSTATE = false;
            //exit(0);
            return "connection_error";

        }

        else
        {
            std::cout << "Error: receive failed ! " <<  std::endl;
            close(ClientSocketSTATE);
            std::cout << "Socket connection closed! Terminating..." << std::endl;
            clientActiveSTATE = false;
            return "connection_error";
            //exit(0);
        }
    }
    else
    {
        std::cout << "Connection to Kuka State client was not possible! Terminating..." << std::endl;
        return "connection_error";
        //exit(0);
    }
}

void sendMessageSTATE(std::string input) {
    if (clientActiveSTATE)
    {
        strcpy(sendbuf_STATE, input.c_str());
        iResult_STATE = send(ClientSocketSTATE, sendbuf_STATE, (int)strlen(sendbuf_STATE), 0);
        if (iResult_STATE == -1)
        {
            std::cout << "Error: send failed !" << std::endl;
            close(ClientSocketSTATE);
            std::cout << "Socket connection closed with Kuka State client!" << std::endl;
            clientActiveSTATE = false;
        }
    }
}

void shutdownSTATE() {
    if (clientActiveSTATE) {
        std::cout << "Info: terminating socket connection by user request..." << std::endl;
        iResult_STATE = shutdown(ClientSocketSTATE, SHUT_WR);
        clientActiveSTATE = false;
        if (iResult_STATE == -1) {
            std::cout << "Error: shutdown failed for Kuka State!" << std::endl;
            close(ClientSocketSTATE);
        }
    }
}
