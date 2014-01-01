#include "globals.h"

SourceType g_source = SOCKETIN;
bool g_paused = false;
bool g_rectify = false;
bool g_stepOne = false;
bool g_detectKeyPoints = false;
bool g_useSurf = false;
bool g_useFilter = false;
bool g_useCPU = false;
bool g_displayStereoWindow = true;
bool g_skipProcessing = true;
bool g_sleep = false;
bool g_showLeft = false;

// right now anything besides my 
StereoMatchMethod  g_stereo = BM;
int  g_ndispMax = 64;
int  g_bmWinSize = 19;
int  g_bThreshold = 0;
int  g_BPiters = 5;
int  g_BPLevels = 5;


unsigned char g_keys[4] = {0};
unsigned char g_speed = 255;
volatile bool g_sendKey = false;

// Histogram to perform some logic descions over a number of frames
// This should ultimately be replaced with a probability map uses
// a kalman filter or the likes there of
int g_descHist[DESC_MAX_HIST];
int g_histSize = 3;
int histPos = 0;