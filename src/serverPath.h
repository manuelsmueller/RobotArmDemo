#pragma once

#ifndef SERVERPATH_H_
#define SERVERPATH_H_

#if defined(__unix__)
#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <vector>
#include "GlobalVariables.h"
#include <string>
typedef int SOCKET;
#define closesocket(i) close(i)
#define ioctlsocket(i,l,ul) ioctl(i,l,ul)
#endif

#define DEFAULT_BUFLEN_PATH 512
#define DEFAULT_PORT_PATH 5000
#define IP_ADDRESS_PATH "192.168.0.102"

void setupServerPATH(bool);

std::string receiveMessagePATH();

void sendMessagePATH(std::string);

void shutdownPATH();

#endif
