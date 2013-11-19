#pragma once

#include "networkConnection.h"
#include <opencv2/gpu/gpu.hpp>
#include <assert.h>

#include "globals.h"
#include <iostream>

class H264StreamSource : public cv::gpu::VideoReader_GPU::VideoSource 
{
public:
    bool started;
    HANDLE inputThread;
    std::string m_vid_server;

    SOCKET ConnectSocket;


    H264StreamSource(std::string serverAddr) :
      inputThread(NULL),
      started(false),
      m_vid_server(serverAddr),
      ConnectSocket(INVALID_SOCKET)
      {}
    ~H264StreamSource() {}

    cv::gpu::VideoReader_GPU::FormatInfo format() const {
        cv::gpu::VideoReader_GPU::FormatInfo ret;
        // hard code for rasppi vid
        ret.chromaFormat = cv::gpu::VideoReader_GPU::YUV420;
        ret.codec = cv::gpu::VideoReader_GPU::H264;
        ret.height = 1080;
        ret.width = 1920;

        return ret;
    }

    int run() {
        const int blockSize = 50000;
        int prevSize = 0;
        unsigned char data[blockSize];
        unsigned char dataNew[blockSize];

        if (g_source == STDIN) 
        {
            HANDLE stdinH = GetStdHandle(STD_INPUT_HANDLE);

            assert(stdinH);
            if (!stdinH)
                return 1;
            DWORD count = 0;
            BOOL res = ReadFile(stdinH, data, blockSize, &count, NULL);

            while (stdinH) 
            {
                //source->
                    //Sleep(1);
                    //count = fread(data, 1, blockSize, dataFile);
                parseVideoData(data, count);
                res = ReadFile(stdinH, data, blockSize, &count, NULL);
            }

        } 
        else
        {
            WSADATA wsaData = {0};
            int iResult = 0;

        //    int i = 1;

            struct addrinfo *result = NULL,
                *ptr = NULL,
                hints;

            ZeroMemory( &hints, sizeof(hints) );
            hints.ai_family = AF_UNSPEC;
            hints.ai_socktype = SOCK_STREAM;
            hints.ai_protocol = IPPROTO_TCP;

            #define DEFAULT_VID_PORT "8080"

            // Resolve the server address and port
            iResult = getaddrinfo(m_vid_server.c_str(), DEFAULT_VID_PORT, &hints, &result);
            if (iResult != 0) {
                printf("getaddrinfo failed: %d\n", iResult);
                WSACleanup();
                return 1;
            }
            ConnectSocket = INVALID_SOCKET;

            // Attempt to connect to the first address returned by
            // the call to getaddrinfo
            ptr=result;

            // Create a SOCKET for connecting to server
            ConnectSocket = socket(ptr->ai_family, ptr->ai_socktype, 
                ptr->ai_protocol);

            if (ConnectSocket == INVALID_SOCKET) {
                printf("Error at socket(): %ld\n", WSAGetLastError());
                freeaddrinfo(result);
                WSACleanup();
                return 1;
            }

            // Connect to server.
            iResult = connect( ConnectSocket, ptr->ai_addr, (int)ptr->ai_addrlen);
            if (iResult == SOCKET_ERROR) {
                closesocket(ConnectSocket);
                ConnectSocket = INVALID_SOCKET;
                return 1;
            }

            freeaddrinfo(result);   

            u_long iMode=1;
            ioctlsocket(ConnectSocket,FIONBIO,&iMode);
            iResult = blockSize;
            // This loop would allow you to discard data and read up to the newest to
            // reduce latency, but then you get decomposition artefacts. Instead
            // later we decompose the frames as fast as possible later

            started = true;
            do {
                while (iResult == blockSize) {
                    iResult = recv(ConnectSocket, (char *)dataNew, blockSize, 0);

                    if ( iResult > 0 )
                    {
                        //printf("Bytes received: %d\n", iResult);
                        
                        memcpy(data,dataNew,iResult);
                        prevSize = iResult;
                        iResult = 1;
                    }
                    else if ( iResult <= 0 )
                    {
                        // we don't have any data
                        int nError=WSAGetLastError();
                        if(nError!=WSAEWOULDBLOCK&&nError!=0)
                        {
                            std::cout<<"Winsock error code: "<<nError<<"\r\n";
                            std::cout<<"Server disconnected!\r\n";
                            // Shutdown our socket
                            shutdown(ConnectSocket,SD_SEND);

                            // Close our socket entirely
                            closesocket(ConnectSocket);

                            return 1;
                        }
                    }
                    else
                    {
                        //printf("recv failed with error: %d\n", WSAGetLastError());
                    }

                }

                assert(prevSize >= 0 && prevSize <= blockSize);
                if (prevSize) 
                {
                    
                    parseVideoData(data, prevSize);
                    // that data is now processed
                    //printf("parsed: %d\n", prevSize);
                    prevSize = 0;
                    
                }

                // start again to recv if possible
                iResult = blockSize;

                if (g_sleep)
                {
                    Sleep(10);
                }

            } while( 1 );

            // cleanup
            closesocket(ConnectSocket);
            WSACleanup();
        }

        //fclose(dataFile);

        return 0;
    }

    void start()  {
        inputThread = CreateThread(NULL, 0, readThread, this, 0, NULL);
    }
    void stop()  {
        //kill thread

    }
    bool isStarted() const {
        return started;
    }
    bool hasError() const {
        return false;
    }

};