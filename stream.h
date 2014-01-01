#pragma once

// network connection must come first so winsock is included before
// windows.h
#include "networkConnection.h"
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <Windows.h>

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/opengl.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2\imgproc\types_c.h>
    
#include <opencv2/nonfree/nonfree.hpp>
#include "h264decoder.h"



class VideoStream {
public:
    VideoStream () :
      m_fname("")
    {

    }


    // H264 stream from socket or stdin
    H264StreamSource m_h264source;
    // CUDA video reader input
    cv::gpu::VideoReader_GPU *m_reader;
    // Webcam input
    cv::VideoCapture webcam;
    // file where the video is loaded from
    std::string m_fname;

    // Source for this stream
    SourceType m_sourceType;

    // Latest frame to process
    // Naming covention g for GPUs frames
    //        and       c for CPUs
    cv::gpu::GpuMat m_gCurrentFrame;
    cv::Mat         m_cCurrentFrame;

    // Gray versions
    cv::gpu::GpuMat m_gCurrentFrameGray;
    cv::Mat         m_cCurrentFrameGray;

    // Rectifying matrices
    //cv::Mat *mx, *my;
    cv::gpu::GpuMat gmx, gmy;

    // Rectified buffers
    cv::gpu::GpuMat m_gCurrentFrameGrayRectified;
    cv::gpu::GpuMat m_gCurrentFrameRectified;

    VideoStream(SourceType source, std::string fname);

    void init(SourceType source, std::string fname);

    bool readLatestFrame();
    bool readOnlyOneLatestFrame();

    bool rectifyCurrentFrame();

    bool bilatralFilter();

    void normalize();

    bool saveLatestFrame(std::string saveFileName);

    bool setRectifyMaps(std::string mapxFileName, std::string mapyFileName);
};

