#include <opencv2/gpu/gpu.hpp>
#include <opencv2\core\cuda\common.hpp>

using namespace cv::gpu::cudev;

__global__   void rgbnorm_dev(float4* image, int pitch)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y; 

    if (x >= 512 || y >= 512)
        return;

    int linearPos = y*pitch + x*4;

    uchar *pixel = ((uchar *)(((char *)image) +linearPos));

    uchar r = pixel[0];
    uchar g = pixel[1];
    uchar b = pixel[2];

    uint sum = r + g + b;

    pixel[0] = (sum > 5 ? (uchar)((double)r/sum*255) : 0);
    pixel[1] = (sum > 5 ? (uchar)((double)g/sum*255) : 0);
    pixel[2] = (sum > 5 ? (uchar)((double)b/sum*255) : 0);
}

void rgbnorm(cv::gpu::GpuMat& image)
{
	dim3 threads(32, 8);
    dim3 grid (divUp (image.cols, threads.x), divUp (image.rows, threads.y));
    
    rgbnorm_dev<<<grid, threads>>>((float4 *)image.ptr(), image.step);

    cudaError_t err ;
    err = cudaDeviceSynchronize();

    if (err != cudaSuccess)
    {
        fprintf(stdout, "Failed to launch vectorAdd kernel (error code %s)!\n", cudaGetErrorString(err));
    }
}

__global__   void mythresh_dev(float4* src, uchar* dst, int pitch, int dstPitch, int*  bounds)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y; 

    if (x >= 512 || y >= 512)
        return;

    int linearPos = y*pitch + x*4;
    int linearPosdst = y*dstPitch + x;
    bool canditate = true;
    uchar *pixel = ((uchar *)(((char *)src) +linearPos));

    int one = bounds[0];
    if (bounds[0] > pixel[0])
        canditate = false;
    if (bounds[1] < pixel[0])
        canditate = false;
    if (bounds[2] > pixel[1])
        canditate = false;
    if (bounds[3] < pixel[1])
        canditate = false;
    if (bounds[4] > pixel[2])
        canditate = false;
    if (bounds[5] < pixel[2])
        canditate = false;
   
    //if (pixel[1] > 50) {
    //    if ((pixel[2] >= pixel[1]-15) && (pixel[2] < pixel[1]+15)) {
    //        canditate = true;
    //        
    //    }

    //   /* if ((pixel[2] >= pixel[0]-40) && (pixel[2] < pixel[0]+40)) {
    //            canditate = true;
    //        }*/
    //}
    dst[linearPosdst] = (canditate ? 255 : 0);

    //dst[linearPosdst] = __float2int_rd(0.5*255);//(((float4 *)(((char *)src) +linearPos))->x*255);
    //else
    //    dst[linearPosdst] = 125;
}


// PtrStepSzb dst
// (PtrStepSz<T>)src

void mythresh(cv::gpu::GpuMat& src, cv::gpu::GpuMat& dst, int bounds[3][2])
{
	dim3 threads(32, 8);
    dim3 grid (divUp (src.cols, threads.x), divUp (src.rows, threads.y));
    cudaError_t err ;
    static int* bounds_arr = NULL;
    if (!bounds_arr) {
        err = cudaMalloc(&bounds_arr,3*2*sizeof(int));
        int data[6];
        memcpy(data,&bounds[0][0],6*4); 
        
    }

    err = cudaMemcpy(bounds_arr, &bounds[0][0],3*2*sizeof(int),cudaMemcpyHostToDevice);

	mythresh_dev<<<grid, threads>>>((float4 *)src.ptr(), dst.ptr(), src.step, dst.step, bounds_arr);


    err = cudaDeviceSynchronize();



    if (err != cudaSuccess)
    {
        fprintf(stdout, "Failed to launch vectorAdd kernel (error code %s)!\n", cudaGetErrorString(err));
    }
}