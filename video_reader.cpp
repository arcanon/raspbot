#undef UNICODE

#define WIN32_LEAN_AND_MEAN
// #define OBJECT_DETECT

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <Windows.h>

#include <cuda_gl_interop.h>
#include <cuda_runtime_api.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2\imgproc\types_c.h>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "networkConnection.h"
#include "h264decoder.h"
#include "stream.h"
#include "stereo.h"
#include "C:/Users/areid/Documents/GitHub/VisionTools/visiontools/stopwatch.h"
#if defined(_WINDOWS)
LARGE_INTEGER VisionTools::StopWatch::perfFreq;
#endif
//#include "e:/GitHub/opencv/modules/gpu/src/frame_queue.h"

using namespace cv;
using namespace cv::gpu;
//using namespace std;

// XXX Global state
#include "globals.h"

void *buttonFunc(int state, void* userdata);

enum ButtonTypes {
    PAUSE_BUTTON,
    KEYPOINT_BUTTON,
    RESTART_BUTTON,
    STEPONE_BUTTON,
    CPU_BUTTON,
    SURF_BUTTON,
    DETECT_BUTTON,
    FILTER_BUTTON,
    RECTIFY_BUTTON,
    LEFTVIEW_BUTTON
};

// various settings
int bounds[3][2];
int match = 950;
int minMatchCount = 20;
int homographyInCount = 5;
int allowedOutliers = 3;
int homoUniqueDist = 7;
int nbMatchCount = 20;
int ransacError = 5;

VideoStream *g_lefteyeStream = NULL;
VideoStream *g_righteyeStream = NULL;

class MyInit 
{
    
public:
    int p;
    MyInit() 
    {
        
    }

    static void Init()
    {

        cv::namedWindow("CPU", cv::WINDOW_NORMAL);
        resizeWindow("CPU",800,600);
        cv::createButton("pause",(cv::ButtonCallback)buttonFunc, (void *) PAUSE_BUTTON,QT_CHECKBOX,0);
        cv::createButton("rectify",(cv::ButtonCallback)buttonFunc, (void *) RECTIFY_BUTTON,QT_CHECKBOX,g_rectify);
        cv::createButton("leftView",(cv::ButtonCallback)buttonFunc, (void *) LEFTVIEW_BUTTON,QT_CHECKBOX,g_showLeft);
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
        cv::createTrackbar("ndispmax",NULL,&g_ndispMax,255);
        cv::createTrackbar("bmwinsize",NULL,&g_bmWinSize,52);
        cv::createTrackbar("stereomethod",NULL,(int *)&g_stereo,2);
        cv::createTrackbar("bpiterations",NULL,(int *)&g_BPiters,50);
        cv::createTrackbar("bplevels",NULL,(int *)&g_BPLevels,50);
        cv::createTrackbar("threshold",NULL,(int *)&g_bThreshold,BM_THRESHOLD_MAX);
        
        //cv::createB
        cv::namedWindow("GPU", cv::WINDOW_OPENGL);
        cudaDeviceReset();
        cv::gpu::setGlDevice();
        resizeWindow("GPU",800,600);

    }
};

MyInit defInits;

class GH
{
public:
    GH (std::string _fname) : 
      surfGpu(200),
          surfCPU(100),
          fname(_fname),
          reader(_fname),
          serverThread(NULL)
      {
          //src = cv::imread("e:\\camera_egs\\lampref2.png");
          src = cv::imread("e:\\camera_egs\\detect.png");
          cvtColor(src, srcGray, CV_RGB2GRAY);
          dst.upload(src);
          detectImage.upload(src);

          cv::gpu::cvtColor(dst, detectImageGray, CV_RGB2GRAY);

          // do you want this???
          //cv::gpu::bilateralFilter(dst,detectImage, 30, 24, 6.5);

          // GPU version does not do this by default, which casues to much variance
          orbGpu.blurForDescriptor = true;

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
      }

    VideoCapture webcam;
    VideoWriter outputVideo;
    std::string fname;
    cv::gpu::VideoReader_GPU *d_reader;
    ORB_GPU orbGpu;
    SURF_GPU surfGpu;
    // GPU defaults to 100 hessian threshold, which seems to cut off when features are accepted, related to image blurr
    SURF surfCPU;
    ORB  orbCPU;

    gpu::GpuMat dst, finalFrame;

    cv::gpu::GpuMat gpuDetectKeyPoints;
    cv::gpu::GpuMat gpuDetectDescriptors;

    HANDLE serverThread;

    Mat cpuDetectDescriptors;
    std::vector<DMatch> matches;

    Mat src;
    Mat srcGray;
    cv::gpu::GpuMat detectImage;
    cv::gpu::GpuMat detectImageGray;

    std::vector<cv::KeyPoint> detectKeyPoints;
    std::vector<cv::KeyPoint> cpuDetectKeyPoints;

    cv::gpu::GpuMat maskFrame;
    Mat cpuMaskFrame;

    cv::VideoCapture reader;

    std::vector<cv::KeyPoint> keyPoints;
    cv::gpu::GpuMat gpuKeyPoints;
    cv::gpu::GpuMat gpuDescriptors;
    Mat cpuDescriptors;
    typedef std::vector<cv::KeyPoint>::iterator keyPointIt;
    typedef std::vector<cv::KeyPoint> keyPointList;

    BFMatcher_GPU matcher; //(g_useSurf ? NORM_L2 : NORM_L1);
    BFMatcher matcherCPU; //(g_useSurf ? NORM_L2 : NORM_L1);

    GpuMat trainIdx, distance;

    cv::Mat frame, drawFrame;

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

    void updateSurfOrbState(int state)
    {
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
    }

    void restart()
    {
        d_reader->close();
        d_reader->open(fname);
    }

    void doObjectDetection()
    {
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
        std::vector<Point2f> mptsTrain, mptsQuery;

        keyPointList &currKeyPoints = g_detectKeyPoints ? detectKeyPoints : keyPoints;
        keyPointList &drawKeyPoints = g_useCPU ? cpuDetectKeyPoints : detectKeyPoints;

        Mat srcOut = src.clone();
        drawFrame = Mat(max(srcOut.rows, frame.rows), srcOut.cols+frame.cols, CV_8U);

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
                        for (std::vector<Point2f>::iterator iter = mptsTrain.begin(); iter != mptsTrain.end(); iter++) {
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
            float diff = 0;
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
                    std::vector<Point2f> output(1);
                    std::vector<Point2f> input(1);
                    input[0] =  Point2f(srcOut.cols/2,srcOut.rows/2);
                    Mat m_output(output);
                    perspectiveTransform(input, m_output, H);
                    cv::circle(drawFrame,output[0]+Point2f(src.cols,0), 200, cv::Scalar(0, 255, 0), 10);


                    // send the direction change
                    printf("output x %f final x %d center x %f", output[0].x, finalFrame.cols/2, center.x);
                    diff = abs((float)finalFrame.cols/2-output[0].x)/((float)finalFrame.cols/2);
                    if (output[0].x < finalFrame.cols/2 - 80)
                    {
                        //g_keys[rightPos] = 1;
                        printf("inst turn right\n");
                        currDesc = -1;

                    }
                    else if (output[0].x > finalFrame.cols/2 +80)
                    {
                        //g_keys[leftPos] = 1;
                        printf("inst turn left\n");
                        currDesc = 1;
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


            unsigned char speed = 255*(0.1 + diff*0.8);

            printf(" speed %d ", speed);
            if (notFound) 
            {
                // default turn left until we find it
                g_keys[leftPos] = 160;
                printf("do nothing\n");
                currDesc = 0xFF;
            }

            g_descHist[histPos] = notFound ? currDesc : currDesc;
            histPos++;
            if (histPos >= g_histSize) 
            {
                histPos = 0;
            }

            // setup keys
            memset(g_keys, 0, sizeof(g_keys));
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
                g_keys[rightPos] = speed;
                printf("hist turn right\n");
            }
            else if (netDir <= -0.4f*g_histSize)
            {
                g_keys[leftPos] = speed;
                printf("hist turn left\n");
            }
            else
            {
                if (notFoundCount > 0.4f*g_histSize)
                {
                    g_keys[leftPos] = 160;
                    printf("not found, turn left\n");
                }
                else
                {
                    printf("hist on target\n");
                }
            }

            /*if (currDesc >= 0.4f) 
            {
            g_keys[rightPos] = 1;
            printf("hist turn right\n");
            }
            else if (netDir <= -0.4f)
            {
            g_keys[leftPos] = 1;
            printf("hist turn left\n");
            }
            else
            {
            if (notFound)
            {
            g_keys[leftPos] = 1;
            printf("not found, turn left\n");
            }
            else
            {
            printf("hist on target\n");
            }
            }*/
            // send keys
            g_sendKey = true;
        }
    }
};


GH * globals = NULL;


static int saveCount = 0;

 void *buttonFunc(int state, void* userdata)
 {
    ButtonTypes type = (ButtonTypes) (int)userdata; 
    switch (type) {
    case SURF_BUTTON:
        globals->updateSurfOrbState(state);
        g_useSurf = !!state;
        break;
    case PAUSE_BUTTON:
        g_paused = !!state;
        break;
    case RECTIFY_BUTTON:
        g_rectify = !!state;
        break;
    case LEFTVIEW_BUTTON:
        g_showLeft = !!state;
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

            globals->restart();

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
        globals->matches.clear();
        g_useCPU  = !!state;
        globals->updateDescriptors();
        break;
    case FILTER_BUTTON:
        g_useFilter = !!state;
        break;
    case DETECT_BUTTON:
        {
            cv::gpu::GpuMat d_frame;
            /*switch (g_source)
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
            }*/

            /*char saveName[100];
            if(g_lefteyeStream)
            {
                sprintf(saveName, "E:/camera_egs/raspi-stereo/imageleft%d.jpg", saveCount);
                g_lefteyeStream->saveLatestFrame(saveName);
            }

            if (g_righteyeStream)
            {
                sprintf(saveName, "E:/camera_egs/raspi-stereo/imageright%d.jpg", saveCount);
                g_righteyeStream->saveLatestFrame(saveName);
            }
            saveCount++;*/
            break;
        }
    }

    return NULL;
 }

void mythresh(cv::gpu::GpuMat& src, cv::gpu::GpuMat& dst, int bounds[3][2]);
void rgbnorm(cv::gpu::GpuMat& image);
void anaglyph(cv::gpu::GpuMat& leftImage, cv::gpu::GpuMat& rightImage, cv::gpu::GpuMat& outputImage);



// all the working variables for object detection
cv::Mat frame;

cv::gpu::GpuMat d_frame;
cv::gpu::GpuMat d_frame2;
cv::gpu::GpuMat d_frame3;
cv::gpu::GpuMat d_frame4(512, 512, CV_8UC1);
H264StreamSource source(vid_server);
cv::TickMeter tm;
std::vector<double> cpu_times;
std::vector<double> gpu_times;



// final frame to show to screen, CPU version
Mat drawFrame;



int main(int argc, const char* argv[])
{
    std::string fname;
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

    MyInit::Init();
    globals = new GH(fname);

    // Start server socket

    globals->serverThread = CreateThread(NULL, 0, serverListenProc, 0, 0, NULL);

    bounds[0][0] = 0;
    bounds[0][1] = 255;
    bounds[1][0] = 0;
    bounds[1][1] = 255;
    bounds[2][0] = 0;
    bounds[2][1] = 255;
    defInits.p = 4;    

    //resizeWindow("CPU",800,600);
    Sleep(50);
    
    
    VideoStream rightEyeStream(g_source, "ashpi2.fritz.box");//"E:\\camera_egs\\video2.h264");
    g_righteyeStream = &rightEyeStream;
    Sleep(50);
    VideoStream leftEyeStream(g_source, "ashpi.fritz.box");
    g_lefteyeStream = &leftEyeStream;

    //set rectify matrices
    leftEyeStream.setRectifyMaps("E:\\camera_egs\\stereo_camera_calibrate\\mx1.xml", "E:\\camera_egs\\stereo_camera_calibrate\\my1.xml");
    rightEyeStream.setRectifyMaps("E:\\camera_egs\\stereo_camera_calibrate\\mx2.xml", "E:\\camera_egs\\stereo_camera_calibrate\\my2.xml");

    // Transform from int to char via Bitwise operators
    //char EXT[] = {(char)(ex & 0XFF) , (char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24), 0};
    // CV_FOURCC('m', 'p', '4', 'v')
    // CV_FOURCC('P','I','M','1')
    bool res = globals->outputVideo.open("e:\\camera_egs\\stereo.avi", CV_FOURCC('m', 'p', '4', 'v'),25,cv::Size(1920,1088));
    assert(res);
    /*switch (g_source)
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
                std::cout << "cannot open camera";
            }
            break;
        }
    }

    if (d_reader) {
        d_reader->dumpFormat(std::cout);
    }*/

    

    gpu::GpuMat d_disp;
    gpu::GpuMat d_color_disp;

    StereoMatching strCtl;

    gpu::GpuMat dst, finalFrame;

    VisionTools::StopWatch loopTime;

#define TIMER_THRESHOLD 0.020

    loopTime.start();
    int count = 0;
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
            //switch (g_source)
            //{
            //    case SOCKETIN:
            //    case STDIN:
            //    {
            //        tm.reset(); tm.start();
            //        // drain queue by reading lots of frames. Note OpenCV was altered to allow this to
            //        // happen. normally Open cv does:
            //        // // Wait a bit
            //        // detail::Thread::sleep(1); 
            //        while (d_reader->read(d_frame2) || !d_frame2.rows)
            //        {
            //            if (!d_frame2.rows)
            //                Sleep(1);
            //            //continue;
            //        }
            //            tm.stop();
            //        gpu_times.push_back(tm.getTimeMilli());
            //        break;
            //    }
            //    
            //    case FILEIN:
            //    {
            //        tm.reset(); tm.start();
            //        if (!d_reader->read(d_frame2))
            //         {
            //            d_reader->close();
            //            d_reader->open(fname);
            //               continue;
            //        }
            //            tm.stop();
            //        gpu_times.push_back(tm.getTimeMilli());
            //        break;
            //    }
            //    case WEBCAM:
            //    {
            //        webcam.read(frame);
            //        d_frame2.upload(frame);
            //        break;
            //    }
            //}

            bool leftres = true;
            bool rightres = true;
            int max_throw = 20;
            bool saveNow = false;
            //leftEyeStream.readLatestFrame();
#define SKIP_MAX 300
            static int savedSkip[2][SKIP_MAX];
            static int savedSkipPos = 0;
            static double savedTime[SKIP_MAX];

            VisionTools::StopWatch captureTime;
            captureTime.start();

            int leftSkip = 0, rightSkip = 0;
            //printf("quque %d\n", rightEyeStream.m_reader->framesInQueue());
            if (rightEyeStream.m_reader->framesInQueue() > 4)
            {
                printf("right overfilled\n");
            }

            if (leftEyeStream.m_reader->framesInQueue() > 4)
            {
                printf("left overfilled\n");
            }

            while (//max_throw &&
                (leftres || rightres)) 
            {
                //if (!rightres)
                {
                    rightres = rightEyeStream.readOnlyOneLatestFrame();
                }
                //rightres = false;
                //if (!leftres)
                {
                    leftres = leftEyeStream.readOnlyOneLatestFrame();
                }
                    //.readOnlyOneLatestFrame();
                max_throw++;

             /*   if ((!leftres || !rightres))
                {
                    Sleep(1);
                }*/

                if (rightres)
                {
                    rightSkip++;
                }

                if (leftres)
                {
                    leftSkip++;
                }

                if (rightres || leftres)
                {
                    saveNow = true;
                }

                //break;
            }
            captureTime.stop();
            if (captureTime.get_stopped_time() > TIMER_THRESHOLD)
            {
                printf("captureTime %4.2f\n", captureTime.get_stopped_time());
            }

            savedSkip[0][savedSkipPos] = leftSkip;
            savedSkip[1][savedSkipPos] = rightSkip;
            loopTime.stop();
            savedTime[savedSkipPos] = loopTime.get_stopped_time();
            if (loopTime.get_stopped_time() > TIMER_THRESHOLD)
            {
                printf("looptime %4.2f\n", loopTime.get_stopped_time());
            }
            loopTime.start();

            

            savedSkipPos++;

            if (false &&
                (savedSkipPos > SKIP_MAX))
            {
                printf("skip data left  ");
                for(int i = 0; i < SKIP_MAX; i++)
                {
                    printf("%1d ", savedSkip[0][i]);
                }
                printf("\nskip data right ");
                for(int i = 0; i < SKIP_MAX; i++)
                {
                    printf("%1d ", savedSkip[1][i]);
                }
               /* printf("\nskip data time  ");
                for(int i = 0; i < SKIP_MAX; i++)
                {
                    printf("%9.2f ", savedTime[i]*1000);
                }*/

                savedSkipPos = 0;
            }

           /* if (saveNow)
            {
                char saveName[100];
                if(g_lefteyeStream)
                {
                    sprintf(saveName, "E:/camera_egs/raspi-stereo/imagealeft%d.jpg", count);
                    g_lefteyeStream->saveLatestFrame(saveName);
                }

                if (g_righteyeStream)
                {
                    sprintf(saveName, "E:/camera_egs/raspi-stereo/imagearight%d.jpg", count);
                    g_righteyeStream->saveLatestFrame(saveName);
                }
                count++;
            }*/

            //leftEyeStream.readLatestFrame();
            //rightEyeStream.readLatestFrame();

            if (g_rectify) 
            {
                leftEyeStream.rectifyCurrentFrame();
                rightEyeStream.rectifyCurrentFrame();
            }

            //finalFrame = cv::gpu::GpuMat(leftEyeStream.m_gCurrentFrameGray.rows, leftEyeStream.m_gCurrentFrameGray.cols, CV_8UC4);
            //anaglyph(leftEyeStream.m_gCurrentFrameGray, rightEyeStream.m_gCurrentFrameGray,finalFrame);

           // gpu::remap(leftEyeStream.m_cCurrentFrameGray,

            if (g_useFilter)
            {
                leftEyeStream.bilatralFilter();
                rightEyeStream.bilatralFilter();
            }

            //switch (g_stereo)
            //{
            //case BM:
            //    {
            //        /*if (leftEyeStream.m_gCurrentFrameGray.size() != d_disp.size())
            //        {
            //            d_disp.create(leftEyeStream.m_gCurrentFrameGray.size(), CV_8U);
            //        }*/
            //        strCtl.getDisparityMapBM(leftEyeStream.m_gCurrentFrameGrayRectified, 
            //                                 rightEyeStream.m_gCurrentFrameGrayRectified,
            //                                 d_disp);
            //       /* cv::gpu::GpuMat tmp;
            //        gpu::normalize(d_disp, tmp, 255);
            //        d_disp = tmp;*/
            //        break;
            //    }
            //case BP:
            //    strCtl.getDisparityMapBP(leftEyeStream.m_gCurrentFrameGray, rightEyeStream.m_gCurrentFrameGray, d_disp);
            //    break;
            //case CSBP:
            //    strCtl.getDisparityMapCSBP(leftEyeStream.m_gCurrentFrameGray, rightEyeStream.m_gCurrentFrameGray, d_disp);
            //    break;
            //default:
            //    assert(0);
            //}

            //cv::gpu::drawColorDisp(d_disp,
            //               d_color_disp,
            //               g_ndispMax);

            //combine left and right into one window  
            // this section of code drops the code to 14fps
            const int frameCols = leftEyeStream.m_gCurrentFrameGray.cols;
            const int frameRows = leftEyeStream.m_gCurrentFrameGray.rows;
            /*if (finalFrame.size() != cv::Size(frameCols*2, frameRows*2))
            {
                finalFrame = cv::gpu::GpuMat(frameRows*2, frameCols*2, CV_8U, cv::Scalar(0));
            }

            cv::gpu::GpuMat subKeyFrame = finalFrame(Rect(0, 0,frameCols, frameRows));

            leftEyeStream.m_gCurrentFrameGray.copyTo(subKeyFrame);

            subKeyFrame = finalFrame(Rect(frameCols, 0,frameCols, frameRows));

            rightEyeStream.m_gCurrentFrameGray.copyTo(subKeyFrame);*/

            /*subKeyFrame = finalFrame(Rect(0, frameRows,frameCols, frameRows));

            d_disp.copyTo(subKeyFrame);

            subKeyFrame = finalFrame(Rect(frameCols, frameRows,frameCols, frameRows));

            d_disp.copyTo(subKeyFrame);*/
            
            //End left and right composition
            

            //cv::gpu::resize(d_frame, d_frame2,cv::Size(512,512));
            //cv::gpu::cvtColor(d_frame2, d_frame, (g_source == WEBCAM ? CV_RGB2GRAY : CV_RGBA2GRAY));

            /*if (g_useFilter)
            {
                cv::gpu::bilateralFilter(d_frame,d_frame2,30,24,6.5);
                finalFrame = d_frame2;
            }
            else
            {
                finalFrame = d_frame;
            }*/

            //finalFrame = leftEyeStream.m_gCurrentFrameGray;

            //finalFrame.download(frame);

            //outputVideo << frame;
            //outputVideo.write(frame);

            //drawFrame = frame;
        }

        if (!g_skipProcessing)
        {
            globals->doObjectDetection();
        }
        
        //rightEyeStream.m_gCurrentFrame = leftEyeStream.m_gCurrentFrame;
        VisionTools::StopWatch displayTime;
        displayTime.start();
        //leftEyeStream.m_gCurrentFrame.download(drawFrame);
        //cv::imshow("CPU", drawFrame);
        cv::imshow("GPU", (g_showLeft ? leftEyeStream.m_gCurrentFrame : rightEyeStream.m_gCurrentFrame));
        displayTime.stop();
        if (displayTime.get_stopped_time() > TIMER_THRESHOLD)
        {
            printf("displayTime %4.2f\n", displayTime.get_stopped_time());
        }
        /*
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
        glColorMask(GL_TRUE, GL_FALSE, GL_FALSE, GL_TRUE);
    
        // set camera for blue eye, red will be filtered.
    
        // draw scene
    
        glClear(GL_DEPTH_BUFFER_BIT);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE);
    
        glColorMask(GL_FALSE, GL_FALSE, GL_TRUE, GL_TRUE);
    
        // set camera for red eye, blue will be filtered.
    
        // draw scene
        */
        VisionTools::StopWatch keyTime;
        keyTime.start();
        int key = cv::waitKey(1);

        switch (key) {
        case 'q':
            globals->outputVideo.release();
            return 0;
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

        if (g_sleep)
        {
            Sleep(10);
        }

        keyTime.stop();
        if (keyTime.get_stopped_time() > TIMER_THRESHOLD)
        {
            printf("keyTime %4.2f\n", keyTime.get_stopped_time());
        }
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

    if (globals->d_reader) 
    {
        delete globals->d_reader;
    }

    delete globals;

    return 0;
}

    