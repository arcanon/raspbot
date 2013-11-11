#include <opencv2/gpu/gpu.hpp>
#include <opencv2\core\cuda\common.hpp>

using namespace cv::gpu::cudev;

__global__   void anaglyph_dev(char* imageLeft, char* imageRight, char *imageOut, int pitchInputs, int pitchOutput, int width, int height)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y; 

    if (x >= width || y >= height)
        return;

    int linearPosInputs = y*pitchInputs + x;
    int linearPosOutput = y*pitchOutput + x*4;

    // Red
    imageOut[linearPosOutput]   = imageLeft[linearPosInputs];
    imageOut[linearPosOutput+1] = 0;//imageRight[linearPosInputs];
    imageOut[linearPosOutput+2] = imageRight[linearPosInputs];
}

void anaglyph(cv::gpu::GpuMat& leftImage, cv::gpu::GpuMat& rightImage, cv::gpu::GpuMat& outputImage)
{
	dim3 threads(32, 8);
    dim3 grid (divUp (leftImage.cols, threads.x), divUp (leftImage.rows, threads.y));
    
    anaglyph_dev<<<grid, threads>>>((char *)leftImage.ptr(), (char *)rightImage.ptr(), (char *)outputImage.ptr(), leftImage.step, outputImage.step, leftImage.cols, leftImage.rows);

    cudaError_t err ;
    err = cudaDeviceSynchronize();

    if (err != cudaSuccess)
    {
        fprintf(stdout, "Failed to launch vectorAdd kernel (error code %s)!\n", cudaGetErrorString(err));
    }
}
