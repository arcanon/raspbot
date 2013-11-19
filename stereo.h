#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv\cv.h>
#include <opencv2/nonfree/gpu.hpp>

class StereoMatching 
{
public:
    StereoMatching();

    //CvMat _M1, _M2, _D1, _D2, _R1, _R2, _P1, _P2;

    //CvMat *mx1, *my1, *mx2, *my2;
    cv::gpu::StereoBM_GPU bm;
    cv::gpu::StereoBeliefPropagation bp;
    cv::gpu::StereoConstantSpaceBP csbp;

    bool getDisparityMapBM(cv::gpu::GpuMat &left, cv::gpu::GpuMat &right, cv::gpu::GpuMat &disp);
    bool getDisparityMapBP(cv::gpu::GpuMat &left, cv::gpu::GpuMat &right, cv::gpu::GpuMat &disp);
    bool getDisparityMapCSBP(cv::gpu::GpuMat &left, cv::gpu::GpuMat &right, cv::gpu::GpuMat &disp);
};
