#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <Windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2\imgproc\types_c.h>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;
using namespace cv::gpu;
using namespace std;

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

#define DEFAULT_BUFLEN 4
#define DEFAULT_PORT "50000"
#define DESC_MAX_HIST 100

DWORD WINAPI readThread(void *params);
DWORD WINAPI serverListenProc(void *params);

enum SourceType {
    STDIN,
    SOCKETIN,
    FILEIN,
    WEBCAM
};

// XXX Global state
SourceType g_source = SOCKETIN;
bool g_paused = false;
bool g_stepOne = false;
bool g_detectKeyPoints = false;
bool g_useSurf = false;
bool g_useFilter = false;
bool g_useCPU = false;

#define forwardPos 0
#define backwardPos 1
#define leftPos 2
#define rightPos 3
char g_keys[4] = {0};
volatile bool g_sendKey = false;
int g_descHist[DESC_MAX_HIST];
int g_histSize = 3;
int histPos = 0;

//char *vid_server = "192.168.43.7";
char *vid_server = "192.168.178.23";

cv::gpu::VideoReader_GPU *d_reader;
VideoCapture webcam;
std::string fname;

ORB_GPU orbGpu;
SURF_GPU surfGpu(200);
// GPU defaults to 100 hessian threshold, which seems to cut off when features are accepted, related to image blurr
SURF surfCPU(100);
ORB  orbCPU;

cv::gpu::GpuMat gpuDetectKeyPoints;
cv::gpu::GpuMat gpuDetectDescriptors;

HANDLE serverThread = NULL;

Mat cpuDetectDescriptors;
vector<DMatch> matches;

Mat src;
Mat srcGray;
cv::gpu::GpuMat detectImage;
cv::gpu::GpuMat detectImageGray;

std::vector<cv::KeyPoint> detectKeyPoints;
std::vector<cv::KeyPoint> cpuDetectKeyPoints;

cv::gpu::GpuMat maskFrame;
Mat cpuMaskFrame;

class H264StreamSource : public cv::gpu::VideoReader_GPU::VideoSource 
{
public:
    bool started;
    HANDLE inputThread;

    H264StreamSource() :
      inputThread(NULL),
      started(false) {}
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
            iResult = getaddrinfo(vid_server, DEFAULT_VID_PORT, &hints, &result);
            if (iResult != 0) {
                printf("getaddrinfo failed: %d\n", iResult);
                WSACleanup();
                return 1;
            }
            SOCKET ConnectSocket = INVALID_SOCKET;

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
                //Sleep(1);

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
        started = true;
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

DWORD WINAPI readThread(void *params) 
{
    H264StreamSource * source = (H264StreamSource *) params;
    source->run();
    return 1;
}

DWORD WINAPI serverListenProc(void *params)
{
     WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    int iSendResult;
    char recvbuf[DEFAULT_BUFLEN];
    int recvbuflen = DEFAULT_BUFLEN;
    
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // Accept a client socket
    ClientSocket = accept(ListenSocket, NULL, NULL);
    if (ClientSocket == INVALID_SOCKET) {
        printf("accept failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    // No longer need server socket
    closesocket(ListenSocket);

    // Receive until the peer shuts down the connection
    do {

        Sleep(1);
        

        if (g_sendKey) {
            //iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
            //if (iResult > 0) {
            //    printf("Bytes received: %d\n", iResult);

            // Echo the buffer back to the sender
            iSendResult = send( ClientSocket, g_keys, 4, 0 );
            if (iSendResult == SOCKET_ERROR) {
                printf("send failed with error: %d\n", WSAGetLastError());
                closesocket(ClientSocket);
                WSACleanup();
                return 1;
            }
            
        
            printf("Bytes sent: %d\n", iSendResult);
            memset(g_keys, 0, sizeof(g_keys));
            g_sendKey = false;

            // cancel the last key
            Sleep(20);
            char tmp[4] = {0,0,0,0};
            iSendResult = send( ClientSocket, tmp, 4, 0 );
            if (iSendResult == SOCKET_ERROR) {
                printf("send failed with error: %d\n", WSAGetLastError());
                closesocket(ClientSocket);
                WSACleanup();
                return 1;
            }
            Sleep(60);
        }

    } while (1);

    // shutdown the connection since we're done
    iResult = shutdown(ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(ClientSocket);
        WSACleanup();
        return 1;
    }

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();


}

enum ButtonTypes {
    PAUSE_BUTTON,
    KEYPOINT_BUTTON,
    RESTART_BUTTON,
    STEPONE_BUTTON,
    CPU_BUTTON,
    SURF_BUTTON,
    DETECT_BUTTON,
    FILTER_BUTTON
};

void updateDescriptors()
{
    if (g_useSurf) 
    {
        surfGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
        surfGpu.downloadKeypoints(gpuDetectKeyPoints, detectKeyPoints);

        cpuDetectDescriptors = Mat();
        surfCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
    }
    else
    {
        orbGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
        orbGpu.downloadKeyPoints(gpuDetectKeyPoints, detectKeyPoints);

        cpuDetectDescriptors = Mat();
        orbCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
    }

    matches.clear();
}

 void *buttonFunc(int state, void* userdata)
 {
    ButtonTypes type = (ButtonTypes) (int)userdata; 
    switch (type) {
    case SURF_BUTTON:
        if (!!state) 
        {
            surfGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
            surfGpu.downloadKeypoints(gpuDetectKeyPoints, detectKeyPoints);

            surfCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
        }
        else
        {
            orbGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
            orbGpu.downloadKeyPoints(gpuDetectKeyPoints, detectKeyPoints);

            orbCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
        }

        g_useSurf = !!state;
        break;
    case PAUSE_BUTTON:
        g_paused = !!state;
        break;
    case KEYPOINT_BUTTON:
        g_detectKeyPoints = !!state;
        break;
    case RESTART_BUTTON:
        {
            bool resume = false;
            if(!g_paused) 
            {
                resume = true;
                g_paused = true;
                Sleep(3000);
            }

            d_reader->close();
            d_reader->open(fname);

            if (resume)
            {
                g_paused = false;
            }
        }

        break;
    case STEPONE_BUTTON:
        g_stepOne = true;
        break;
    case CPU_BUTTON:
        matches.clear();
        g_useCPU  = !!state;
        updateDescriptors();
        break;
    case FILTER_BUTTON:
        g_useFilter = !!state;
        break;
    case DETECT_BUTTON:
        {
            cv::gpu::GpuMat d_frame;
            switch (g_source)
            {
            case SOCKETIN:
            case STDIN:
            case FILEIN:
                {
                    if (!d_reader->read(detectImage))
                    {
                        assert(!"bad");
                    }
                    else
                    {
                        detectImage.download(src);
                        detectImageGray.download(srcGray);
                    }
                    break;
                }
            case WEBCAM:
                {
                    if (!webcam.read(src))
                    {
                        assert(!"bad");
                    } 
                    else
                    {
                        src = src.clone();
                        cvtColor(src, srcGray, CV_RGB2GRAY);
                        detectImage.upload(src);
                    }
                    break;
                }
            }

            imwrite( "e:\\camera_egs\\detect.png", src );

            cv::gpu::cvtColor(detectImage, detectImageGray, (g_source == WEBCAM ? CV_RGB2GRAY : CV_RGBA2GRAY));
            cvtColor(src, srcGray, CV_RGB2GRAY);
            if (g_useSurf) 
            {
                surfGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
                surfGpu.downloadKeypoints(gpuDetectKeyPoints, detectKeyPoints);

                surfCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
            }
            else
            {
                orbGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
                orbGpu.downloadKeyPoints(gpuDetectKeyPoints, detectKeyPoints);

                orbCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
            }
            break;
        }
    }

    return NULL;
 }

void mythresh(cv::gpu::GpuMat& src, cv::gpu::GpuMat& dst, int bounds[3][2]);
void rgbnorm(cv::gpu::GpuMat& image);

int main(int argc, const char* argv[])
{

    if (g_source != WEBCAM) {
        if (argc != 2) 
        {
            //g_source = STDIN;
            fname = "";
        }
        else 
        {
            g_source = FILEIN;
            fname = argv[1];
        }
    }

    // Start server socket

    serverThread = CreateThread(NULL, 0, serverListenProc, 0, 0, NULL);

    int bounds[3][2];
    int match = 950;
    int minMatchCount = 20;
    int homographyInCount = 5;
    int allowedOutliers = 3;
    int homoUniqueDist = 7;
    int nbMatchCount = 20;
    int ransacError = 5;
    bounds[0][0] = 0;
    bounds[0][1] = 255;
    bounds[1][0] = 0;
    bounds[1][1] = 255;
    bounds[2][0] = 0;
    bounds[2][1] = 255;

    cv::namedWindow("CPU", cv::WINDOW_NORMAL);
    resizeWindow("CPU",800,600);
    cv::createButton("pause",(cv::ButtonCallback)buttonFunc, (void *) PAUSE_BUTTON,QT_CHECKBOX,0);
    cv::createButton("key point",(cv::ButtonCallback)buttonFunc, (void *) KEYPOINT_BUTTON,QT_CHECKBOX,0);
    cv::createButton("surf",(cv::ButtonCallback)buttonFunc, (void *) SURF_BUTTON,QT_CHECKBOX,g_useSurf);
    cv::createButton("cpu",(cv::ButtonCallback)buttonFunc, (void *) CPU_BUTTON,QT_CHECKBOX,g_useCPU);
    cv::createButton("filter",(cv::ButtonCallback)buttonFunc, (void *) FILTER_BUTTON,QT_CHECKBOX,g_useFilter);
    cv::createButton("restart",(cv::ButtonCallback)buttonFunc, (void *) RESTART_BUTTON,QT_PUSH_BUTTON,0);
    cv::createButton("stepone",(cv::ButtonCallback)buttonFunc, (void *) STEPONE_BUTTON,QT_PUSH_BUTTON,0);
    cv::createButton("savedetect",(cv::ButtonCallback)buttonFunc, (void *) DETECT_BUTTON,QT_PUSH_BUTTON,0);

    cv::createTrackbar("r-lower",NULL,&bounds[0][0],255);
    cv::createTrackbar("r-lower",NULL,&bounds[0][0],255);
    cv::createTrackbar("r-higher",NULL,&bounds[0][1],255);
    cv::createTrackbar("g-lower",NULL,&bounds[1][0],255);
    cv::createTrackbar("g-higher",NULL,&bounds[1][1],255);
    cv::createTrackbar("b-lower",NULL,&bounds[2][0],255);
    cv::createTrackbar("b-higher",NULL,&bounds[2][1],255);
    cv::createTrackbar("match",NULL,&match,3000);
    cv::createTrackbar("minmatchcount",NULL,&minMatchCount,50);
    cv::createTrackbar("nbMatchCount",NULL,&nbMatchCount,50);
    cv::createTrackbar("homographyInCount",NULL,&homographyInCount,20);
    cv::createTrackbar("allowedOutliers",NULL,&allowedOutliers,20);
    cv::createTrackbar("homoUniqueDist",NULL,&homoUniqueDist,100);
    cv::createTrackbar("ransacError",NULL,&ransacError,50);
    cv::createTrackbar("histSize",NULL,&g_histSize,DESC_MAX_HIST);

    //cv::createB
    

    //cv::namedWindow("GPU", cv::WINDOW_OPENGL);



    //cv::gpu::setGlDevice();

    cv::Mat frame;
    cv::VideoCapture reader(fname);

    cv::gpu::GpuMat d_frame;
    cv::gpu::GpuMat d_frame2;
    cv::gpu::GpuMat d_frame3;
    cv::gpu::GpuMat d_frame4(512, 512, CV_8UC1);
    H264StreamSource source;
    
    switch (g_source)
    {
        case SOCKETIN:
        case STDIN:
        {
            d_reader = new cv::gpu::VideoReader_GPU(cv::Ptr<H264StreamSource>(&source));
            break;
        }
        case FILEIN:
        {
            d_reader = new cv::gpu::VideoReader_GPU(fname);
            break;
        }
        case WEBCAM:
        {
            webcam.open(0);
            webcam.set(CAP_PROP_FRAME_WIDTH, 800);
            webcam.set(CAP_PROP_FRAME_HEIGHT, 600);
            if  (!webcam.isOpened()) 
            {
                cout << "cannot open camera";
            }
            break;
        }
    }

    if (d_reader) {
        d_reader->dumpFormat(std::cout);
    }

    cv::TickMeter tm;
    std::vector<double> cpu_times;
    std::vector<double> gpu_times;

    std::vector<cv::KeyPoint> keyPoints;
    cv::gpu::GpuMat gpuKeyPoints;
    cv::gpu::GpuMat gpuDescriptors;
    Mat cpuDescriptors;
    typedef std::vector<cv::KeyPoint>::iterator keyPointIt;
    typedef std::vector<cv::KeyPoint> keyPointList;


    //src = cv::imread("e:\\camera_egs\\lampref2.png");
    src = cv::imread("e:\\camera_egs\\detect.png");
    cvtColor(src, srcGray, CV_RGB2GRAY);
    gpu::GpuMat dst, finalFrame;

    dst.upload(src);
    detectImage.upload(src);

    
    //orbGpu.blurForDescriptor = true;
    BFMatcher_GPU matcher(g_useSurf ? NORM_L2 : NORM_L1);
    BFMatcher matcherCPU(g_useSurf ? NORM_L2 : NORM_L1);

    cv::gpu::cvtColor(dst, detectImageGray, CV_RGB2GRAY);
    cv::gpu::bilateralFilter(dst,detectImage,30,24,6.5);
    if (g_useSurf) 
    {
        surfGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
        surfGpu.downloadKeypoints(gpuDetectKeyPoints, detectKeyPoints);

        surfCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
    }
    else
    {
        orbGpu(detectImageGray, maskFrame, gpuDetectKeyPoints, gpuDetectDescriptors);
        orbGpu.downloadKeyPoints(gpuDetectKeyPoints, detectKeyPoints);

        orbCPU(srcGray, cpuMaskFrame, cpuDetectKeyPoints, OutputArray(cpuDetectDescriptors));
    }

    GpuMat trainIdx, distance;
    for (;;)
    {
        //tm.reset(); tm.start();
        //if (!reader.read(frame))
        //{
        //    reader.release();
        //    reader.open(fname);
        //    continue;
        //}
        //    //break;
        //tm.stop();
        //cpu_times.push_back(tm.getTimeMilli());

        if (!g_paused || g_stepOne)
        {
            switch (g_source)
            {
                case SOCKETIN:
                case STDIN:
                {
                    tm.reset(); tm.start();
                    // drain queue by reading lots of frames. Note OpenCV was altered to allow this to
                    // happen. normally Open cv does:
                    // // Wait a bit
                    // detail::Thread::sleep(1); 
                    while (d_reader->read(d_frame2) || !d_frame2.rows)
                    {
                        if (!d_frame2.rows)
                            Sleep(1);
                        //continue;
                    }
                        tm.stop();
                    gpu_times.push_back(tm.getTimeMilli());
                    break;
                }
                
                case FILEIN:
                {
                    tm.reset(); tm.start();
                    if (!d_reader->read(d_frame2))
                     {
                        d_reader->close();
                        d_reader->open(fname);
                           continue;
                    }
                        tm.stop();
                    gpu_times.push_back(tm.getTimeMilli());
                    break;
                }
                case WEBCAM:
                {
                    webcam.read(frame);
                    d_frame2.upload(frame);
                    break;
                }
            }

            //cv::gpu::resize(d_frame, d_frame2,cv::Size(512,512));
            cv::gpu::cvtColor(d_frame2, d_frame, (g_source == WEBCAM ? CV_RGB2GRAY : CV_RGBA2GRAY));

            if (g_useFilter)
            {
                cv::gpu::bilateralFilter(d_frame,d_frame2,30,24,6.5);
                finalFrame = d_frame2;
            }
            else
            {
                finalFrame = d_frame;
            }

            finalFrame.download(frame);

            
        }

        if (!g_useCPU) 
        {
            if (g_useSurf) 
            {
                surfGpu(finalFrame, maskFrame, gpuKeyPoints, gpuDescriptors);
                surfGpu.downloadKeypoints(gpuKeyPoints, keyPoints);
            }
            else
            {
                orbGpu(finalFrame, maskFrame, gpuKeyPoints, gpuDescriptors);
                orbGpu.downloadKeyPoints(gpuKeyPoints, keyPoints);
            }

            matcher.matchSingle(gpuDescriptors, gpuDetectDescriptors, trainIdx, distance);
            matcher.matchDownload(trainIdx, distance, matches);
        } 
        else 
        {
            if (g_useSurf) 
            {
                surfCPU(frame, cpuMaskFrame, keyPoints, OutputArray(cpuDescriptors));
            }
            else
            {
                orbCPU(frame, cpuMaskFrame, keyPoints, OutputArray(cpuDescriptors));
            }

            if (cpuDetectDescriptors.type() != cpuDescriptors.type()) 
            {
                updateDescriptors();
            }

            if (cpuDetectDescriptors.total()) 
            {
                matcherCPU.match(cpuDescriptors, cpuDetectDescriptors, matches);
            } 
            else
            {
                matches.clear();
            }
        }


        std::sort(matches.begin(),matches.end());

        if (g_stepOne) 
        {
            g_stepOne = false;
        }

        //cv::gpu::resize(d_frame, d_frame2,cv::Size(512,512));

        //rgbnorm(d_frame2);

        //cv::gpu::bilateralFilter(d_frame2,d_frame3,9,18,4.5);

        //mythresh(d_frame2, d_frame4, bounds);

        //cv::imshow("GPU", d_frame2);

        int nbMatches = std::min((unsigned int)nbMatchCount,matches.size());
        vector<Point2f> mptsTrain, mptsQuery;
        
        keyPointList &currKeyPoints = g_detectKeyPoints ? detectKeyPoints : keyPoints;
        keyPointList &drawKeyPoints = g_useCPU ? cpuDetectKeyPoints : detectKeyPoints;

        Mat srcOut = src.clone();
        Mat drawFrame = Mat(max(srcOut.rows, frame.rows), srcOut.cols+frame.cols, CV_8U);

        Mat subKeyFrame =drawFrame(Rect(0, 0,srcOut.cols, srcOut.rows));
        
        srcGray.copyTo(subKeyFrame);

        subKeyFrame = drawFrame(Rect(srcOut.cols, 0,frame.cols, frame.rows));

        frame.copyTo(subKeyFrame);
        //subKeyFrame.setTo(src);

        //subKeyFrame = drawFrame.adjustROI(0, srcOut.rows+frame.rows, 0, srcOut.cols+frame.cols);

        //Mat subKeyFrame = drawFrame(Range(0, srcOut.rows), Range(0, srcOut.cols));
            

        
        //drawFrame = g_detectKeyPoints ? srcOut : frame;
        
       /* for (keyPointIt i = currKeyPoints.begin(); i != currKeyPoints.end(); i++) {
            cv::circle(drawFrame, (*i).pt, 10, cv::Scalar(0, 0, 255));
        }*/

        if (drawKeyPoints.size() != 0) 
        {
            // Draw all points on the image
            
            for (int i = 0; i < drawKeyPoints.size(); i++) 
            {
                cv::circle(drawFrame, (drawKeyPoints[i]).pt, 10, cv::Scalar(0, 0, 255));
            }

            bool falseDetection = false;
            Point2f center;
            int matchCount = 0;


            //std::vector<cv::KeyPoint> &currTrainKeyPoints = (g_useCPU ? cpuDetectKeyPoints : detectKeyPoints)
            for (int i = 0; i < nbMatches; i++) {
                 int index = g_detectKeyPoints ? matches[i].trainIdx : matches[i].queryIdx;
                 if (index < currKeyPoints.size() && (matches[i].distance < (float) match / (g_useSurf ? 500 : 1))) {
                     if ( mptsTrain.size() < homographyInCount ) {
                        bool tooClose = false;
                        for (vector<Point2f>::iterator iter = mptsTrain.begin(); iter != mptsTrain.end(); iter++) {
                            Point2f diff = drawKeyPoints[matches[i].trainIdx].pt - *iter;
                            if (sqrt(diff.ddot(diff)) < homoUniqueDist) 
                            {
                                tooClose = true;
                            }
                        }
                        if (!tooClose) 
                        {
                            mptsTrain.push_back(drawKeyPoints[matches[i].trainIdx].pt);
                            mptsQuery.push_back(keyPoints[matches[i].queryIdx].pt);
                        }
                    }
                     int radius = 10;

                     if (g_useSurf)
                     {
                          radius = 10+(matches[i].distance > 1 ? ((matches[i].distance-1)/2)*500 +10:0);
                     }
                     else
                     {
                         radius = 10+(matches[i].distance > 300 ? (matches[i].distance-300)/10:0);
                     }

                     //cv::circle(drawFrame, (drawKeyPoints[matches[i].trainIdx]).pt, radius, cv::Scalar(0, 0, 255));
                     cv::circle(drawFrame, (keyPoints[matches[i].queryIdx]).pt+Point2f(src.cols,0), radius, cv::Scalar(0, 0, 255));
                     cv::line(drawFrame,(drawKeyPoints[matches[i].trainIdx]).pt, (keyPoints[matches[i].queryIdx]).pt+Point2f(src.cols,0), cv::Scalar(0, 0, 255), 2);
                     center += (currKeyPoints[index]).pt;
                     matchCount++;
                 } else {
                     //falseDetection = true;
                 }
            }
            int currDesc = 0;
            center.x /= matchCount;
            center.y /= matchCount;
            bool notFound=true;
            std::vector<uchar> outliers(100);
            if (!falseDetection && matchCount && (matchCount >= minMatchCount)) {
                if (mptsTrain.size() < 4) 
                {
                    mptsTrain.push_back(mptsTrain[0]);
                    mptsQuery.push_back(mptsQuery[0]);
                }
                cv::Mat H = findHomography(mptsTrain,
                    mptsQuery,
                    RANSAC,
                    ransacError,
                    outliers);
                cv::circle(drawFrame, center+Point2f(src.cols,0), 200, cv::Scalar(255, 0, 0), 10);
                if (countNonZero(Mat(outliers)) > allowedOutliers)
                {
                    vector<Point2f> output(1);
                    vector<Point2f> input(1);
                    input[0] =  Point2f(srcOut.cols/2,srcOut.rows/2);
                    Mat m_output(output);
                    perspectiveTransform(input, m_output, H);
                    cv::circle(drawFrame,output[0]+Point2f(src.cols,0), 200, cv::Scalar(0, 255, 0), 10);

                    
                    // send the direction change
                    printf("output x %f final x %d center x %f", output[0].x, finalFrame.cols/2, center.x);
                    if (output[0].x < finalFrame.cols/2 - 80)
                    {
                        //g_keys[rightPos] = 1;
                        printf("inst turn right\n");
                        currDesc = 1;

                    }
                    else if (output[0].x > finalFrame.cols/2 +80)
                    {
                        //g_keys[leftPos] = 1;
                        printf("inst turn left\n");
                        currDesc = -1;
                    }
                    else
                    {
                        printf("inst on target\n");
                        currDesc = 0;
                    }

                    //g_sendKey = true;
                    notFound = false;
                }
            
            }

            if (notFound) 
            {
                // default turn left until we find it
                g_keys[leftPos] = 1;
                printf("do nothing\n");
                currDesc = 0xFF;
            }

            g_descHist[histPos] = currDesc;
            histPos++;
            if (histPos >= g_histSize) 
            {
                histPos = 0;
            }

            // setup keys
            int netDir = 0;
            int notFoundCount = 0;
            for (int i = 0; i < g_histSize; i++)
            {
                if (g_descHist[i] != 0xff)
                {
                    netDir += g_descHist[i];
                }
                else
                {
                    notFoundCount++;
                }
            }

            if (netDir >= 0.4f*g_histSize) 
            {
                g_keys[rightPos] = 1;
                printf("hist turn right\n");
            }
            else if (netDir <= -0.4f*g_histSize)
            {
                g_keys[leftPos] = 1;
                printf("hist turn left\n");
            }
            else
            {
                if (notFoundCount > 0.4f*g_histSize)
                {
                    g_keys[leftPos] = 1;
                    printf("not found, turn left\n");
                }
                else
                {
                    printf("hist on target\n");
                }
            }
            // send keys
            g_sendKey = true;
        }

        cv::imshow("CPU", drawFrame);

        int key = cv::waitKey(1);

        switch (key) {
        case 'w':
            g_keys[forwardPos] = 1;
            g_sendKey = true;
            break;
        case 's':
            g_keys[backwardPos] = 1;
            g_sendKey = true;
            break;
        case 'a':
            g_keys[leftPos] = 1;
            g_sendKey = true;
            break;
        case 'd':
            g_keys[rightPos] = 1;
            g_sendKey = true;
            break;

        }
        //if (cv::waitKey(3) > 0)
            //break;
    }

    if (!cpu_times.empty() && !gpu_times.empty())
    {
        std::cout << std::endl << "Results:" << std::endl;

        std::sort(cpu_times.begin(), cpu_times.end());
        std::sort(gpu_times.begin(), gpu_times.end());

        double cpu_avg = std::accumulate(cpu_times.begin(), cpu_times.end(), 0.0) / cpu_times.size();
        double gpu_avg = std::accumulate(gpu_times.begin(), gpu_times.end(), 0.0) / gpu_times.size();

        std::cout << "CPU : Avg : " << cpu_avg << " ms FPS : " << 1000.0 / cpu_avg << std::endl;
        std::cout << "GPU : Avg : " << gpu_avg << " ms FPS : " << 1000.0 / gpu_avg << std::endl;
    }

    if (d_reader) 
    {
        delete d_reader;
    }

    return 0;
}

    