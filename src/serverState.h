#pragma once

#ifndef SERVERSTATE_H_
#define SERVERSTATE_H_

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

#define DEFAULT_BUFLEN_STATE 128
#define DEFAULT_PORT_STATE 5500
#define IP_ADDRESS_STATE "192.168.0.102"


void setupServerSTATE(bool);

std::string receiveMessageSTATE();

void sendMessageSTATE(std::string);

void shutdownSTATE();

#endif
