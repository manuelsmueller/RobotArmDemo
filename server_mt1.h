/*
filename: server_mt1.h
version 1.0 2022-02-24

description:
The header file server_ml.h is responsible for the communication between the visualization component and
the first physical Monitoring Tool. The server sends commands requesting the physical environment state
and receives a list of detected patterns and their position relative to the Monitoring Tool in return.

(c) 2003-2022 IAS, Universit�t Stuttgart
*/

// FOR DETAILED CODE EXPLANATION, VISIT SEVER_ML.H
// -----------------------------------------------

#pragma once

#define DEFAULT_BUFLEN_MT1 512
#define DEFAULT_PORT_MT1 "4000"
#define IP_ADDRESS_MT1 "192.168.1.9"

WSADATA wsaData_MT1;
int iResult_MT1;

SOCKET ListenSocketMT1 = INVALID_SOCKET;
SOCKET ClientSocketMT1 = INVALID_SOCKET;

struct addrinfo* resultMT1 = NULL;
struct addrinfo hintsMT1;

char recvbuf_MT1[DEFAULT_BUFLEN_MT1] = "";
int recvbuflen_MT1 = DEFAULT_BUFLEN_MT1;

char sendbuf_MT1[DEFAULT_BUFLEN_MT1];

bool clientActiveMT1 = false;

void setupServerMT1() {
    iResult_MT1 = WSAStartup(MAKEWORD(2, 2), &wsaData_MT1);
    if (iResult_MT1 != 0) {
        cout << "Error: WSAStartup failed!" << endl;
    }
    else {
        ZeroMemory(&hintsMT1, sizeof(hintsMT1));
        hintsMT1.ai_family = AF_INET;
        hintsMT1.ai_socktype = SOCK_STREAM;
        hintsMT1.ai_protocol = IPPROTO_TCP;
        hintsMT1.ai_flags = AI_PASSIVE;

        iResult_MT1 = getaddrinfo(IP_ADDRESS_MT1, DEFAULT_PORT_MT1, &hintsMT1, &resultMT1);
        if (iResult_MT1 != 0) {
            cout << "Error: getaddrinfo failed with error!" << endl;
            WSACleanup();
            exit(0);
        }
        else {
            // cout << "Info: gettaddrinfo successful" << endl;
            ListenSocketMT1 = socket(resultMT1->ai_family, resultMT1->ai_socktype, resultMT1->ai_protocol);
            if (ListenSocketMT1 == INVALID_SOCKET) {
                cout << "Error: socket creation failed: " << WSAGetLastError() << "!" << endl;
                freeaddrinfo(resultMT1);
                WSACleanup();
                exit(0);
            }
            else {
                // cout << "Info: socket successful" << endl;
                iResult_MT1 = bind(ListenSocketMT1, resultMT1->ai_addr, (int)resultMT1->ai_addrlen);
                if (iResult_MT1 == SOCKET_ERROR) {
                    cout << "Error: socket binding failed: " << WSAGetLastError() << "!" << endl;
                    freeaddrinfo(resultMT1);
                    closesocket(ListenSocketMT1);
                    WSACleanup();
                    exit(0);
                }
                else {
                    // cout << "Info: bind successful" << endl;
                    freeaddrinfo(resultMT1);

                    iResult_MT1 = listen(ListenSocketMT1, SOMAXCONN);
                    if (iResult_MT1 == SOCKET_ERROR) {
                        cout << "Error: listening for client failed: " << WSAGetLastError() << "!" << endl;
                        closesocket(ListenSocketMT1);
                        WSACleanup();
                        exit(0);
                    }
                    else {
                        cout << "Info: Monitoring Tool 1 server is waiting for client..." << endl;
                        ClientSocketMT1 = accept(ListenSocketMT1, NULL, NULL);
                        if (ClientSocketMT1 == INVALID_SOCKET) {
                            cout << "Error: connection accept failed: " << WSAGetLastError() << "!" << endl;
                            closesocket(ListenSocketMT1);
                            WSACleanup();
                            exit(0);
                        }
                        else {
                            cout << "Info: successfully connected to Monitoring Tool 1 client" << endl;
                            closesocket(ListenSocketMT1);
                            clientActiveMT1 = true;
                        }
                    }
                }
            }
        }
    }
    cout << endl;
}

void receiveMessageMT1() {
    if (clientActiveMT1) {
        memset(&recvbuf_MT1[0], 0, sizeof(recvbuf_MT1));
        iResult_MT1 = recv(ClientSocketMT1, recvbuf_MT1, recvbuflen_MT1, 0);
        if (iResult_MT1 > 0) {
            stateMessageMT1 = string(recvbuf_MT1);
        }
        else if (iResult_MT1 == 0) {
            cout << "Error: socket connection closed! Terminating..." << endl;
            clientActiveMT1 = false;
            stateMessageMT1 = "connection_error";
            exit(0);
        }
        else {
            cout << "Error: receive failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketMT1);
            WSACleanup();
            cout << "Socket connection closed! Terminating..." << endl;
            clientActiveMT1 = false;
            stateMessageMT1 = "connection_error";
            exit(0);
        }
    }
    else {
        cout << "Connection to Monitoring Tool 1 client was not possible! Terminating..." << endl;
        stateMessageMT1 = "connection_error";
        exit(0);
    }
}

void sendMessageMT1(string input) {
    if (clientActiveMT1) {
        strcpy(sendbuf_MT1, input.c_str());
        iResult_MT1 = send(ClientSocketMT1, sendbuf_MT1, (int)strlen(sendbuf_MT1), 0);
        if (iResult_MT1 == SOCKET_ERROR) {
            cout << "Error: send failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketMT1);
            WSACleanup();
            cout << "Socket connection closed!" << endl;
            clientActiveMT1 = false;
        }
    }
}

void shutdownMT1() {
    if (clientActiveMT1) {
        cout << "Info: terminating socket connection by user request..." << endl;
        iResult_MT1 = shutdown(ClientSocketMT1, SD_SEND);
        clientActiveMT1 = false;
        if (iResult_MT1 == SOCKET_ERROR) {
            cout << "Error: shutdown failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketMT1);
            WSACleanup();
        }
    }
}