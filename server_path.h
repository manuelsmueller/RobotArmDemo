/*
filename: server_path.h
version 1.0 2022-02-24

description:
The header file server_ml.h is responsible for the communication between the visualization and
PathExecutionComponent in the ROS environment. The server sends path informations to the Path-
ExecutionComponent and receives confirmations in return.

(c) 2003-2022 IAS, Universität Stuttgart
*/

// FOR DETAILED CODE EXPLANATION, VISIT SEVER_ML.H
// -----------------------------------------------

#pragma once

#define DEFAULT_BUFLEN_PATH 512
#define DEFAULT_PORT_PATH "5000"
#define IP_ADDRESS_PATH "192.168.0.7"

WSADATA wsaData_PATH;
int iResult_PATH;

SOCKET ListenSocketPATH = INVALID_SOCKET;
SOCKET ClientSocketPATH = INVALID_SOCKET;

struct addrinfo* resultPATH = NULL;
struct addrinfo hintsPATH;

char recvbuf_PATH[DEFAULT_BUFLEN_PATH] = "";
int recvbuflen_PATH = DEFAULT_BUFLEN_PATH;

char sendbuf_PATH[DEFAULT_BUFLEN_PATH];

bool clientActivePATH = false;

void setupServerPATH(bool virtualMode) { // the parameter virtualMode lets the user select between the control of the physical
                                         // robot or a virtual representation instead for example for debugging purposes
    iResult_PATH = WSAStartup(MAKEWORD(2, 2), &wsaData_PATH);
    if (iResult_PATH != 0) {
        cout << "Error: WSAStartup failed!" << endl;
    }
    else {
        ZeroMemory(&hintsPATH, sizeof(hintsPATH));
        hintsPATH.ai_family = AF_INET;
        hintsPATH.ai_socktype = SOCK_STREAM;
        hintsPATH.ai_protocol = IPPROTO_TCP;
        hintsPATH.ai_flags = AI_PASSIVE;

        if (virtualMode) iResult_PATH = getaddrinfo("192.168.1.68", DEFAULT_PORT_PATH, &hintsPATH, &resultPATH);
        else iResult_PATH = getaddrinfo(IP_ADDRESS_PATH, DEFAULT_PORT_PATH, &hintsPATH, &resultPATH);

        if (iResult_PATH != 0) {
            cout << "Error: getaddrinfo failed with error!" << endl;
            WSACleanup();
            exit(0);
        }
        else {
            // cout << "Info: gettaddrinfo successful" << endl;
            ListenSocketPATH = socket(resultPATH->ai_family, resultPATH->ai_socktype, resultPATH->ai_protocol);
            if (ListenSocketPATH == INVALID_SOCKET) {
                cout << "Error: socket creation failed: " << WSAGetLastError() << "!" << endl;
                freeaddrinfo(resultPATH);
                WSACleanup();
                exit(0);
            }
            else {
                // cout << "Info: socket successful" << endl;
                iResult_PATH = bind(ListenSocketPATH, resultPATH->ai_addr, (int)resultPATH->ai_addrlen);
                if (iResult_PATH == SOCKET_ERROR) {
                    cout << "Error: socket binding failed: " << WSAGetLastError() << "!" << endl;
                    freeaddrinfo(resultPATH);
                    closesocket(ListenSocketPATH);
                    WSACleanup();
                    exit(0);
                }
                else {
                    // cout << "Info: bind successful" << endl;
                    freeaddrinfo(resultPATH);

                    iResult_PATH = listen(ListenSocketPATH, SOMAXCONN);
                    if (iResult_PATH == SOCKET_ERROR) {
                        cout << "Error: listening for client failed: " << WSAGetLastError() << "!" << endl;
                        closesocket(ListenSocketPATH);
                        WSACleanup();
                        exit(0);
                    }
                    else {
                        cout << "Info: Kuka path server is waiting for client..." << endl;
                        ClientSocketPATH = accept(ListenSocketPATH, NULL, NULL);
                        if (ClientSocketPATH == INVALID_SOCKET) {
                            cout << "Error: connection accept failed: " << WSAGetLastError() << "!" << endl;
                            closesocket(ListenSocketPATH);
                            WSACleanup();
                            exit(0);
                        }
                        else {
                            cout << "Info: successfully connected to Kuka path client" << endl;
                            closesocket(ListenSocketPATH);
                            clientActivePATH = true;
                        }
                    }
                }
            }
        }
    }
    cout << endl;
}

void receiveMessagePATH() {
    if (clientActivePATH) {
        memset(&recvbuf_PATH[0], 0, sizeof(recvbuf_PATH));
        iResult_PATH = recv(ClientSocketPATH, recvbuf_PATH, recvbuflen_PATH, 0);
        if (iResult_PATH > 0) {
            stateMessagePATH = string(recvbuf_PATH);
        }
        else if (iResult_PATH == 0) {
            cout << "Error: socket connection closed! Terminating..." << endl;
            clientActivePATH = false;
            stateMessagePATH = "connection_error";
            exit(0);
        }
        else {
            cout << "Error: receive failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketPATH);
            WSACleanup();
            cout << "Socket connection closed! Terminating..." << endl;
            clientActivePATH = false;
            stateMessagePATH = "connection_error";
            exit(0);
        }
    }
    else {
        cout << "Connection to Kuka path client was not possible! Terminating..." << endl;
        stateMessagePATH = "connection_error";
        exit(0);
    }
}

void sendMessagePATH(string input) {
    if (clientActivePATH) {
        strcpy(sendbuf_PATH, input.c_str());
        iResult_PATH = send(ClientSocketPATH, sendbuf_PATH, (int)strlen(sendbuf_PATH), 0);
        if (iResult_PATH == SOCKET_ERROR) {
            cout << "Error: send failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketPATH);
            WSACleanup();
            cout << "Socket connection closed!" << endl;
            clientActivePATH = false;
        }
    }
}

void shutdownPATH() {
    if (clientActivePATH) {
        cout << "Info: terminating socket connection by user request..." << endl;
        iResult_PATH = shutdown(ClientSocketPATH, SD_SEND);
        clientActivePATH = false;
        if (iResult_PATH == SOCKET_ERROR) {
            cout << "Error: shutdown failed with: " << WSAGetLastError() << endl;
            closesocket(ClientSocketPATH);
            WSACleanup();
        }
    }
}