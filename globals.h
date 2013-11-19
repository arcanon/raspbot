#pragma once

enum SourceType {
    STDIN,
    SOCKETIN,
    FILEIN,
    WEBCAM
};

// global shared variables
extern SourceType g_source;
extern bool       g_paused;
extern bool g_rectify;
extern bool g_sleep;
extern bool g_stepOne;
extern bool g_detectKeyPoints;
extern bool g_useSurf;
extern bool g_useFilter;
extern bool g_useCPU;
extern bool g_displayStereoWindow;
extern bool g_skipProcessing;
extern int  g_bThreshold;
#define BM_THRESHOLD_MAX 1000

// Stereo constants

enum StereoMatchMethod 
{
    BM = 0,
    BP = 1,
    CSBP = 2
};
extern StereoMatchMethod  g_stereo;
extern int  g_ndispMax;
extern int  g_bmWinSize;
extern int  g_BPiters;
extern int  g_BPLevels;

// Key command definitions
#define forwardPos 0
#define backwardPos 1
#define leftPos 2
#define rightPos 3

// Current decsition of server to send to PI
extern unsigned char g_keys[4];
extern unsigned char g_speed;
extern volatile bool g_sendKey;

// Histogram to perform some logic descions over a number of frames
// This should ultimately be replaced with a probability map uses
// a kalman filter or the likes there of
#define DESC_MAX_HIST 100
extern int g_descHist[DESC_MAX_HIST];
extern int g_histSize;
extern int histPos;