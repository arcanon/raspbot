#include "stereo.h"
#include "globals.h"

StereoMatching::StereoMatching()
{
    /*mx1 = (CvMat*)cvLoad("mx1.xml");
    my1 = (CvMat*)cvLoad("my1.xml");
    mx2 = (CvMat*)cvLoad("mx2.xml");
    my2 = (CvMat*)cvLoad("my2.xml");*/
}

bool StereoMatching::getDisparityMapBM(cv::gpu::GpuMat &left, cv::gpu::GpuMat &right, cv::gpu::GpuMat &disp)
{
    bm.preset = cv::gpu::StereoBM_GPU::BASIC_PRESET;
    bm.ndisp = g_ndispMax;
    bm.winSize = g_bmWinSize;
    bm.avergeTexThreshold = (float)g_bThreshold/BM_THRESHOLD_MAX*50;
    if (left.size() != disp.size())
    {
        disp.create(left.size(), CV_8U);
    }
    bm(left, right, disp);


    return true;
}

bool StereoMatching::getDisparityMapBP(cv::gpu::GpuMat &left, cv::gpu::GpuMat &right, cv::gpu::GpuMat &disp)
{
    bp.ndisp = g_ndispMax;
    bp.iters = g_BPiters;
    bp.levels = g_BPLevels;
    if (left.size() != disp.size())
    {
        disp.create(left.size(), CV_16S);
    }
    bp(left, right, disp);

    return true;
}

bool StereoMatching::getDisparityMapCSBP(cv::gpu::GpuMat &left, cv::gpu::GpuMat &right, cv::gpu::GpuMat &disp)
{
    csbp.ndisp = g_ndispMax;
    csbp.iters = g_BPiters;
    csbp.levels = g_BPLevels;
    if (left.size() != disp.size())
    {
        disp.create(left.size(), CV_16S);
    }
    csbp(left, right, disp);

    return true;
}


// Need to call gpu::remap to get the rectified images and then stereoBM