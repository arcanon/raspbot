#pragma once

#if !defined(WIN32_LEAN_AND_MEAN)
#define WIN32_LEAN_AND_MEAN
#endif

#include <Windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include "globals.h"

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

// server to raspi command client
#define DEFAULT_BUFLEN 4
#define DEFAULT_PORT "50000"

// readThread is responsible for reading the video input and handing it over to CUDA
// for decompression
DWORD WINAPI readThread(void *params);
DWORD WINAPI serverListenProc(void *params);

// Address of video server
//char *vid_server = "192.168.43.7";
//char *vid_server = "192.168.178.23";
//char *vid_server = "ashley.za.net";
static char *vid_server = "ashpi.fritz.box";

// Running raspivid:
// raspivid -ss 5500 -t 999999 -b 2000000 -w 1920 -h 1080 -o - | nc -l -p 8080 


