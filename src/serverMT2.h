#pragma once

#ifndef SERVERMT2_H_
#define SERVERMT2_H_

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

#define DEFAULT_BUFLEN_MT2 512
#define DEFAULT_PORT_MT2 4500
#define IP_ADDRESS_MT2 "192.168.0.102"

void setupServerMT2();

std::string receiveMessageMT2();

void sendMessageMT2(std::string);

void shutdownMT2();



#endif
