#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>


__global__  void person_threshold(unsigned char * pix, int cols, int count){
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	unsigned char p = (unsigned char)0;
	float ff;
	 if(i < count - cols ){
             int diffX =  ((int)pix[i+ cols] - (int)pix[i + cols- 1]);
             int diffY =  ((int)pix[i + cols] - (int)pix[i]);     
             p = (unsigned char)((int)sqrtf((diffX * diffX) + (diffY * diffY)));
__syncthreads();
	pix[i] = p;
}
__syncthreads();
if( i> 0 && count - cols-1){
	p = (pix[i-1] +  pix[i+1] + pix[i + cols] + pix[i+1 + cols])/12;
}
__syncthreads();
 pix[i] +=p;

if( i > 0 && i < count - cols - 1){
	p = (pix[i-1] +  pix[i+1] + pix[i + cols] + pix[i+1 + cols])/12;
}
__syncthreads();
 pix[i] +p;

if( i > 0 &&i < count - cols - 1){
	p = (pix[i-1] +  pix[i+1] + pix[i + cols] + pix[i+1 + cols])/12;
}
__syncthreads();
 pix[i] +=p;

int radius = 1;
if(i > cols + 1  && i < count - cols - 1 && i % cols != 0){	
	p = (
abs(pix[i-1-cols] - pix[i]) 
+ abs(pix[i-cols] - pix[i]) 
+ abs(pix[i + 1 - cols] - pix[i]) 
+ abs(pix[i -1] - pix[i]) + abs( pix[i + 1] -  pix[i]) 
+ abs(pix[i - 1 + cols] - pix[i])
 + abs(pix[i + cols] - pix[i]) 
+ abs(pix[i + 1 + cols] - pix[i])
)/8;
}

__syncthreads();
 pix[i] = p;
}
void  edgedetect(unsigned char * pic, int count, int cols){
     const int numthreads = count;
     const int blocks = numthreads/512;
     const int block_width =  512;
     unsigned char * data = NULL;
	cudaMalloc(&data, count * sizeof(unsigned char));

	cudaMemcpy(data, pic, count* sizeof(unsigned char), cudaMemcpyHostToDevice);

	
        person_threshold<<<blocks, block_width >>>(data, cols, count);

	cudaMemcpy(pic, data, count * sizeof(unsigned char), cudaMemcpyDeviceToHost);

	cudaFree(data);
}
