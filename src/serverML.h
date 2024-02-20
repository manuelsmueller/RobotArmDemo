#pragma once
#ifndef SERVERML_H_
#define SERVERMLH_

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

#define DEFAULT_BUFLEN_ML 675
#define DEFAULT_PORT_ML 3500
#define IP_ADDRESS_ML "127.0.0.1"



void setupServerML();

std::string receiveMessageML();

void sendMessageML(std::string);

void shutdownML();

#endif 
