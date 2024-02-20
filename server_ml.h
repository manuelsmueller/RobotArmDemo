#pragma once

#if defined(__unix__)
#include <sys/socket.h>
typedef int SOCKET;
#define closesocket(i) close(i)
#define ioctlsocket(i,l,ul) ioctl(i,l,ul)
#endif

#define DEFAULT_BUFLEN_ML 675
#define DEFAULT_PORT_ML "3500"
#define IP_ADDRESS_ML "127.0.0.1"

WSADATA wsaData_ML;
int iResult_ML;

SOCKET ListenSocketML = INVALID_SOCKET;
SOCKET ClientSocketML = INVALID_SOCKET;

struct addrinfo* resultML = NULL;
struct addrinfo hintsML;

char recvbuf_ML[DEFAULT_BUFLEN_ML] = "";
int recvbuflen_ML = DEFAULT_BUFLEN_ML;

char sendbuf_ML[DEFAULT_BUFLEN_ML];

bool clientActiveML = false;

void setupServerML() {
    iResult_ML = WSAStartup(MAKEWORD(2, 2), &wsaData_ML);
    if (iResult_ML != 0) {
        cout << "Error: WSAStartup failed!" << endl;
    }
    else {
        ZeroMemory(&hintsML, sizeof(hintsML));
        hintsML.ai_family = AF_INET;
        hintsML.ai_socktype = SOCK_STREAM;
        hintsML.ai_protocol = IPPROTO_TCP;
        hintsML.ai_flags = AI_PASSIVE;

        iResult_ML = getaddrinfo(IP_ADDRESS_ML, DEFAULT_PORT_ML, &hintsML, &resultML);
        if (iResult_ML != 0) {
            cout << "Error: getaddrinfo failed with error!" << endl;
            WSACleanup();
            exit(0);
        }
        else {
            // cout << "Info: gettaddrinfo successful" << endl;
            ListenSocketML = socket(resultML->ai_family, resultML->ai_socktype, resultML->ai_protocol);
            if (ListenSocketML == INVALID_SOCKET) {
                cout << "Error: socket creation failed: " << WSAGetLastError() << "!" << endl;
                freeaddrinfo(resultML);
                WSACleanup();
                exit(0);
            }
            else {
                // cout << "Info: socket successful" << endl;
                iResult_ML = bind(ListenSocketML, resultML->ai_addr, (int)resultML->ai_addrlen);
                if (iResult_ML == SOCKET_ERROR) {
                    cout << "Error: socket binding failed: " << WSAGetLastError() << "!" << endl;
                    freeaddrinfo(resultML);
                    closesocket(ListenSocketML);
                    WSACleanup();
                    exit(0);
                }
                else {
                    // cout << "Info: bind successful" << endl;
                    freeaddrinfo(resultML);

                    iResult_ML = listen(ListenSocketML, SOMAXCONN);
                    if (iResult_ML == SOCKET_ERROR) {
                        cout << "Error: listening for client failed: " << WSAGetLastError() << "!" << endl;
                        closesocket(ListenSocketML);
                        WSACleanup();
                        exit(0);
                    }
                    else {
                        cout << "Info: Machine Learning server is waiting for client..." << endl;
                        ClientSocketML = accept(ListenSocketML, NULL, NULL);
                        if (ClientSocketML == INVALID_SOCKET) {
                            cout << "Error: connection accept failed: " << WSAGetLastError() << "!" << endl;
                            closesocket(ListenSocketML);
                            WSACleanup();
                            exit(0);
                        }
                        else {
                            cout << "Info: successfully connected to Machine Learning client" << endl;
                            closesocket(ListenSocketML);
                            clientActiveML = true;
                        }
                    }
                }
            }
        }
    }
    cout << endl;
}

string receiveMessageML() {
    if (clientActiveML) {
        memset(&recvbuf_ML[0], 0, sizeof(recvbuf_ML));
        iResult_ML = recv(ClientSocketML, recvbuf_ML, recvbuflen_ML, 0);
        if (iResult_ML > 0) {
            return string(recvbuf_ML);
        }
        else if (iResult_ML == 0) {
            cout << "Error: socket connection closed! Terminating..." << endl;
            clientActiveML = false;
            exit(0);
            return "connection_error";
        }
        else {
            cout << "Error: receive failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketML);
            WSACleanup();
            cout << "Socket connection closed! Terminating..." << endl;
            clientActiveML = false;
            exit(0);
            return "connection_error";
        }
    }
    else {
        cout << "Connection to Machine Learning client was not possible! Terminating..." << endl;
        exit(0);
        return "connection_error";
    }
}

void sendMessageML(string input) {
    if (clientActiveML) {
        strcpy(sendbuf_ML, input.c_str());
        iResult_ML = send(ClientSocketML, sendbuf_ML, (int)strlen(sendbuf_ML), 0);
        if (iResult_ML == SOCKET_ERROR) {
            cout << "Error: send failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketML);
            WSACleanup();
            cout << "Socket connection closed!" << endl;
            clientActiveML = false;
        }
    }
}

void shutdownML() {
    if (clientActiveML) {
        cout << "Info: terminating socket connection by user request..." << endl;
        iResult_ML = shutdown(ClientSocketML, SD_SEND);
        clientActiveML = false;
        if (iResult_ML == SOCKET_ERROR) {
            cout << "Error: shutdown failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketML);
            WSACleanup();
        }
    }
}
