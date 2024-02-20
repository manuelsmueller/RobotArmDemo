/*
filename: server_state.h
version 1.0 2022-02-24

description:
The header file server_ml.h is responsible for the communication between the visualization and
AbortAndStateComponent in the ROS environment. The server sends robot pose requests and abort
commands to the AbortAndStateComponent and receives path and pose informations or confirmations
in return.

(c) 2003-2022 IAS, Universit�t Stuttgart
*/

// FOR DETAILED CODE EXPLANATION, VISIT SEVER_ML.H
// -----------------------------------------------

#pragma once

#define DEFAULT_BUFLEN_STATE 128
#define DEFAULT_PORT_STATE "5500"
#define IP_ADDRESS_STATE "192.168.0.7"

WSADATA wsaData_STATE;
int iResult_STATE;

SOCKET ListenSocketSTATE = INVALID_SOCKET;
SOCKET ClientSocketSTATE = INVALID_SOCKET;

struct addrinfo* resultSTATE = NULL;
struct addrinfo hintsSTATE;

char recvbuf_STATE[DEFAULT_BUFLEN_STATE] = "";
int recvbuflen_STATE = DEFAULT_BUFLEN_STATE;

char sendbuf_STATE[DEFAULT_BUFLEN_STATE];

bool clientActiveSTATE = false;

void setupServerSTATE(bool virtualMode) {
    iResult_STATE = WSAStartup(MAKEWORD(2, 2), &wsaData_STATE);
    if (iResult_STATE != 0) {
        cout << "Error: WSAStartup failed!" << endl;
    }
    else {
        ZeroMemory(&hintsSTATE, sizeof(hintsSTATE));
        hintsSTATE.ai_family = AF_INET;
        hintsSTATE.ai_socktype = SOCK_STREAM;
        hintsSTATE.ai_protocol = IPPROTO_TCP;
        hintsSTATE.ai_flags = AI_PASSIVE;

        iResult_STATE = getaddrinfo(IP_ADDRESS_STATE, DEFAULT_PORT_STATE, &hintsSTATE, &resultSTATE);

        if (virtualMode) iResult_STATE = getaddrinfo("192.168.1.68", DEFAULT_PORT_STATE, &hintsSTATE, &resultSTATE);
        else iResult_STATE = getaddrinfo(IP_ADDRESS_STATE, DEFAULT_PORT_STATE, &hintsSTATE, &resultSTATE);

        if (iResult_STATE != 0) {
            cout << "Error: getaddrinfo failed with error!" << endl;
            WSACleanup();
            exit(0);
        }
        else {
            // cout << "Info: gettaddrinfo successful" << endl;
            ListenSocketSTATE = socket(resultSTATE->ai_family, resultSTATE->ai_socktype, resultSTATE->ai_protocol);
            if (ListenSocketSTATE == INVALID_SOCKET) {
                cout << "Error: socket creation failed: " << WSAGetLastError() << "!" << endl;
                freeaddrinfo(resultSTATE);
                WSACleanup();
                exit(0);
            }
            else {
                // cout << "Info: socket successful" << endl;
                iResult_STATE = bind(ListenSocketSTATE, resultSTATE->ai_addr, (int)resultSTATE->ai_addrlen);
                if (iResult_STATE == SOCKET_ERROR) {
                    cout << "Error: socket binding failed: " << WSAGetLastError() << "!" << endl;
                    freeaddrinfo(resultSTATE);
                    closesocket(ListenSocketSTATE);
                    WSACleanup();
                    exit(0);
                }
                else {
                    // cout << "Info: bind successful" << endl;
                    freeaddrinfo(resultSTATE);

                    iResult_STATE = listen(ListenSocketSTATE, SOMAXCONN);
                    if (iResult_STATE == SOCKET_ERROR) {
                        cout << "Error: listening for client failed: " << WSAGetLastError() << "!" << endl;
                        closesocket(ListenSocketSTATE);
                        WSACleanup();
                        exit(0);
                    }
                    else {
                        cout << "Info: Kuka state server is waiting for client..." << endl;
                        ClientSocketSTATE = accept(ListenSocketSTATE, NULL, NULL);
                        if (ClientSocketSTATE == INVALID_SOCKET) {
                            cout << "Error: connection accept failed: " << WSAGetLastError() << "!" << endl;
                            closesocket(ListenSocketSTATE);
                            WSACleanup();
                            exit(0);
                        }
                        else {
                            cout << "Info: successfully connected to Kuka state client" << endl;
                            closesocket(ListenSocketSTATE);
                            clientActiveSTATE = true;
                        }
                    }
                }
            }
        }
    }
    cout << endl;
}

void receiveMessageSTATE() {
    if (clientActiveSTATE) {
        memset(&recvbuf_STATE[0], 0, sizeof(recvbuf_STATE));
        iResult_STATE = recv(ClientSocketSTATE, recvbuf_STATE, recvbuflen_STATE, 0);
        if (iResult_STATE > 0) {
            stateMessageSTATE = string(recvbuf_STATE);
        }
        else if (iResult_STATE == 0) {
            cout << "Error: socket connection closed! Terminating..." << endl;
            clientActiveSTATE = false;
            stateMessageSTATE = "connection_error";
            exit(0);
        }
        else {
            cout << "Error: receive failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketSTATE);
            WSACleanup();
            cout << "Socket connection closed! Terminating..." << endl;
            clientActiveSTATE = false;
            stateMessageSTATE = "connection_error";
            exit(0);
        }
    }
    else {
        cout << "Connection to Kuka state client was not possible! Terminating..." << endl;
        stateMessageSTATE = "connection_error";
        exit(0);
    }
}

void sendMessageSTATE(string input) {
    if (clientActiveSTATE) {
        strcpy(sendbuf_STATE, input.c_str());
        iResult_STATE = send(ClientSocketSTATE, sendbuf_STATE, (int)strlen(sendbuf_STATE), 0);
        if (iResult_STATE == SOCKET_ERROR) {
            cout << "Error: send failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketSTATE);
            WSACleanup();
            cout << "Socket connection closed!" << endl;
            clientActiveSTATE = false;
        }
    }
}

void shutdownSTATE() {
    if (clientActiveSTATE) {
        cout << "Info: terminating socket connection by user request..." << endl;
        iResult_STATE = shutdown(ClientSocketSTATE, SD_SEND);
        clientActiveSTATE = false;
        if (iResult_STATE == SOCKET_ERROR) {
            cout << "Error: shutdown failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketSTATE);
            WSACleanup();
        }
    }
}