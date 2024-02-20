
#include "serverML.h"


int iResult_ML;

int ListenSocketML , ClientSocketML;

struct sockaddr_in server_addressML;
struct sockaddr_in client_addressML;

char recvbuf_ML[DEFAULT_BUFLEN_ML] = "";
int recvbuflen_ML = DEFAULT_BUFLEN_ML;

char sendbuf_ML[DEFAULT_BUFLEN_ML];


bool clientActiveML = false;

void setupServerML() {

   ListenSocketML = socket(AF_INET, SOCK_STREAM, 0);
   if(ListenSocketML < 0)
   {

        std::cout<< "Error: socket creation failed for ML server!" << std::endl;
        exit(0);

   }

    memset(&server_addressML, 0, sizeof(server_addressML));
    server_addressML.sin_family = AF_INET;
    server_addressML.sin_port = htons(DEFAULT_PORT_ML);
    inet_pton(AF_INET, IP_ADDRESS_ML, &server_addressML.sin_addr);


    if(bind(ListenSocketML, (struct sockaddr*)&server_addressML, sizeof(server_addressML)) <0)
    {
        std::cout<< "Error: socket binding failed for Machine Learning server!"<<std::endl;
        exit(0);

    }

    if(listen(ListenSocketML, SOMAXCONN)< 0)
    {
        std::cout<< "Error: Listening for Machine Learning failed!"<<std::endl;
        exit(0);
    }

    std::cout<< "Info: Machine Learning server is waiting for client....." <<std::endl;


    socklen_t client_len = sizeof(client_addressML);
    ClientSocketML = accept(ListenSocketML, (struct sockaddr*) &client_addressML, &client_len);
    if(ClientSocketML < 0)
    {
        std::cout<< "Error: connection accept failed to Machine Learning server!"<< std::endl;
        exit(0);
    }

    std::cout<< "Info: Successfully connected to Machine Learning Client" <<std::endl;
    close(ListenSocketML);
    clientActiveML = true;

    std::cout << std::endl;
}

std::string receiveMessageML() {
    if (clientActiveML)
    {
        memset(&recvbuf_ML[0], 0, sizeof(recvbuf_ML));
        int iResult_ML = recv(ClientSocketML, recvbuf_ML, recvbuflen_ML, 0);

        if (iResult_ML > 0)
        {
            return(std::string(recvbuf_ML));
        }
        else if (iResult_ML == 0)
        {
            std::cout << "Error: socket connection closed! Terminating..." << std::endl;
            clientActiveML = false;
            //exit(0);
            return "connection_error";

        }

        else
        {
            std::cout << "Error: receive failed ! " <<  std::endl;
            close(ClientSocketML);
            std::cout << "Socket connection closed! Terminating..." << std::endl;
            clientActiveML = false;
            return "connection_error";
            //exit(0);
        }
    }
    else
    {
        std::cout << "Connection to Machine Learning client was not possible! Terminating..." << std::endl;
        return "connection_error";
        //exit(0);
    }
}

void sendMessageML(std::string input) {
    if (clientActiveML)
    {
        strcpy(sendbuf_ML, input.c_str());
        int iResult_ML = send(ClientSocketML, sendbuf_ML, (int)strlen(sendbuf_ML), 0);
        if (iResult_ML == -1)
        {
            std::cout << "Error: send failed !" << std::endl;
            close(ClientSocketML);
            std::cout << "Socket connection closed with Machine Learning client!" << std::endl;
            clientActiveML = false;
        }
    }
}

void shutdownML() {
    if (clientActiveML) {
        std::cout << "Info: terminating socket connection by user request..." << std::endl;
        int iResult_ML = shutdown(ClientSocketML, SHUT_WR);
        clientActiveML = false;
        if (iResult_ML == -1) {
            std::cout << "Error: shutdown failed for Machine Learning!" << std::endl;
            close(ClientSocketML);
        }
    }
}

