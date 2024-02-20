#pragma once
#ifndef SERVERMT1_H_
#define SERVERMT1_H_

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

#define DEFAULT_BUFLEN_MT1 512
#define DEFAULT_PORT_MT1 4000
#define IP_ADDRESS_MT1 "192.168.0.102"


void setupServerMT1();

std::string receiveMessageMT1();

void sendMessageMT1(std::string);

void shutdownMT1();
#endif
