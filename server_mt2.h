/*
filename: server_mt2.h
version 1.0 2022-02-24

description:
The header file server_ml.h is responsible for the communication between the visualization component and
the second physical Monitoring Tool. The server sends commands requesting the physical environment state
and receives a list of detected patterns and their position relative to the Monitoring Tool in return.

(c) 2003-2022 IAS, Universität Stuttgart
*/

// FOR DETAILED CODE EXPLANATION, VISIT SEVER_ML.H
// -----------------------------------------------

#pragma once

#define DEFAULT_BUFLEN_MT2 512
#define DEFAULT_PORT_MT2 "4500"
#define IP_ADDRESS_MT2 "192.168.0.7"

WSADATA wsaData_MT2;
int iResult_MT2;

SOCKET ListenSocketMT2 = INVALID_SOCKET;
SOCKET ClientSocketMT2 = INVALID_SOCKET;

struct addrinfo* resultMT2 = NULL;
struct addrinfo hintsMT2;

char recvbuf_MT2[DEFAULT_BUFLEN_MT2] = "";
int recvbuflen_MT2 = DEFAULT_BUFLEN_MT2;

char sendbuf_MT2[DEFAULT_BUFLEN_MT2];

bool clientActiveMT2 = false;

void setupServerMT2() {
    iResult_MT2 = WSAStartup(MAKEWORD(2, 2), &wsaData_MT2);
    if (iResult_MT2 != 0) {
        cout << "Error: WSAStartup failed!" << endl;
    }
    else {
        ZeroMemory(&hintsMT2, sizeof(hintsMT2));
        hintsMT2.ai_family = AF_INET;
        hintsMT2.ai_socktype = SOCK_STREAM;
        hintsMT2.ai_protocol = IPPROTO_TCP;
        hintsMT2.ai_flags = AI_PASSIVE;

        iResult_MT2 = getaddrinfo(IP_ADDRESS_MT2, DEFAULT_PORT_MT2, &hintsMT2, &resultMT2);
        if (iResult_MT2 != 0) {
            cout << "Error: getaddrinfo failed with error!" << endl;
            WSACleanup();
            exit(0);
        }
        else {
            // cout << "Info: gettaddrinfo successful" << endl;
            ListenSocketMT2 = socket(resultMT2->ai_family, resultMT2->ai_socktype, resultMT2->ai_protocol);
            if (ListenSocketMT2 == INVALID_SOCKET) {
                cout << "Error: socket creation failed: " << WSAGetLastError() << "!" << endl;
                freeaddrinfo(resultMT2);
                WSACleanup();
                exit(0);
            }
            else {
                // cout << "Info: socket successful" << endl;
                iResult_MT2 = bind(ListenSocketMT2, resultMT2->ai_addr, (int)resultMT2->ai_addrlen);
                if (iResult_MT2 == SOCKET_ERROR) {
                    cout << "Error: socket binding failed: " << WSAGetLastError() << "!" << endl;
                    freeaddrinfo(resultMT2);
                    closesocket(ListenSocketMT2);
                    WSACleanup();
                    exit(0);
                }
                else {
                    // cout << "Info: bind successful" << endl;
                    freeaddrinfo(resultMT2);

                    iResult_MT2 = listen(ListenSocketMT2, SOMAXCONN);
                    if (iResult_MT2 == SOCKET_ERROR) {
                        cout << "Error: listening for client failed: " << WSAGetLastError() << "!" << endl;
                        closesocket(ListenSocketMT2);
                        WSACleanup();
                        exit(0);
                    }
                    else {
                        cout << "Info: Monitoring Tool 2 server is waiting for client..." << endl;
                        ClientSocketMT2 = accept(ListenSocketMT2, NULL, NULL);
                        if (ClientSocketMT2 == INVALID_SOCKET) {
                            cout << "Error: connection accept failed: " << WSAGetLastError() << "!" << endl;
                            closesocket(ListenSocketMT2);
                            WSACleanup();
                            exit(0);
                        }
                        else {
                            cout << "Info: successfully connected to Monitoring Tool 2 client" << endl;
                            closesocket(ListenSocketMT2);
                            clientActiveMT2 = true;
                        }
                    }
                }
            }
        }
    }
    cout << endl;
}

void receiveMessageMT2() {
    if (clientActiveMT2) {
        memset(&recvbuf_MT2[0], 0, sizeof(recvbuf_MT2));
        iResult_MT2 = recv(ClientSocketMT2, recvbuf_MT2, recvbuflen_MT2, 0);
        if (iResult_MT2 > 0) {
            stateMessageMT2 = string(recvbuf_MT2);
        }
        else if (iResult_MT2 == 0) {
            cout << "Error: socket connection closed! Terminating..." << endl;
            clientActiveMT2 = false;
            stateMessageMT2 = "connection_error";
            exit(0);
        }
        else {
            cout << "Error: receive failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketMT2);
            WSACleanup();
            cout << "Socket connection closed! Terminating..." << endl;
            clientActiveMT2 = false;
            stateMessageMT2 = "connection_error";
            exit(0);
        }
    }
    else {
        cout << "Connection to Monitoring Tool 2 client was not possible! Terminating..." << endl;
        stateMessageMT2 = "connection_error";
        exit(0);
    }
}

void sendMessageMT2(string input) {
    if (clientActiveMT2) {
        strcpy(sendbuf_MT2, input.c_str());
        iResult_MT2 = send(ClientSocketMT2, sendbuf_MT2, (int)strlen(sendbuf_MT2), 0);
        if (iResult_MT2 == SOCKET_ERROR) {
            cout << "Error: send failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketMT2);
            WSACleanup();
            cout << "Socket connection closed!" << endl;
            clientActiveMT2 = false;
        }
    }
}

void shutdownMT2() {
    if (clientActiveMT2) {
        cout << "Info: terminating socket connection by user request..." << endl;
        iResult_MT2 = shutdown(ClientSocketMT2, SD_SEND);
        clientActiveMT2 = false;
        if (iResult_MT2 == SOCKET_ERROR) {
            cout << "Error: shutdown failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketMT2);
            WSACleanup();
        }
    }
}