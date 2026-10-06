#include "semantic_voxel_mapping/trace.hpp"
#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <chrono>
namespace svm {
inline void check(cudaError_t error){if(error!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(error));}
template<class T>struct Buffer {
 T* ptr=nullptr;explicit Buffer(size_t count){if(count)check(cudaMalloc(&ptr,count*sizeof(T)));}
 ~Buffer(){if(ptr)cudaFree(ptr);}Buffer(const Buffer&)=delete;Buffer& operator=(const Buffer&)=delete;
};
__global__ void count_rays(const Ray* rays,unsigned* counts,size_t n,double r,double lo,double hi){auto i=size_t(blockIdx.x)*blockDim.x+threadIdx.x;if(i<n)counts[i]=trace(rays[i],r,lo,hi,nullptr);}
__global__ void emit_rays(const Ray* rays,const unsigned* offsets,Key* keys,size_t n,double r,double lo,double hi){auto i=size_t(blockIdx.x)*blockDim.x+threadIdx.x;if(i<n)trace(rays[i],r,lo,hi,keys+offsets[i]);}
void require_cuda(int device){int n;check(cudaGetDeviceCount(&n));if(device<0||device>=n)throw std::runtime_error("CUDA device unavailable");check(cudaSetDevice(device));check(cudaFree(nullptr));}
GpuResult raycast_cuda(const std::vector<Ray>& rays,double r,double lo,double hi,size_t budget){
 GpuResult result;std::unordered_set<Key> free;const size_t maximum_chunk=16384;
 for(size_t base=0;base<rays.size();){
  size_t n=std::min(maximum_chunk,rays.size()-base);unsigned bound=0;
  // Budget conservatively covers input, prefix arrays, output and radix-sort scratch.
  for(size_t i=base;i<base+n;++i){auto& a=rays[i].origin;auto& b=rays[i].end;
   unsigned steps=unsigned(std::ceil((fabs(b.x-a.x)+fabs(b.y-a.y)+fabs(b.z-a.z))/r))+6;bound=std::max(bound,steps);}
  size_t per_ray=sizeof(Ray)+sizeof(unsigned)*2+sizeof(Key)*size_t(bound)*3;
  n=std::min(n,budget>8?(budget-8)/per_ray:0);if(n<1)throw std::runtime_error("CUDA scratch budget too small for one ray");
  result.peak_scratch_bytes=std::max(result.peak_scratch_bytes,n*per_ray+8);
  Buffer<Ray> input(n);Buffer<unsigned> counts(n+1),offsets(n+1);
  check(cudaMemcpy(input.ptr,rays.data()+base,n*sizeof(Ray),cudaMemcpyHostToDevice));check(cudaMemset(counts.ptr+n,0,sizeof(unsigned)));
  cudaEvent_t begin{},end{};check(cudaEventCreate(&begin));check(cudaEventCreate(&end));
  try {
   check(cudaEventRecord(begin));count_rays<<<(n+255)/256,256>>>(input.ptr,counts.ptr,n,r,lo,hi);check(cudaGetLastError());
   thrust::exclusive_scan(thrust::device_pointer_cast(counts.ptr),thrust::device_pointer_cast(counts.ptr+n+1),thrust::device_pointer_cast(offsets.ptr));
   unsigned total;check(cudaMemcpy(&total,offsets.ptr+n,sizeof(total),cudaMemcpyDeviceToHost));
   if(size_t(total)>n*bound)throw std::runtime_error("Unexpected ray output overflow");
   Buffer<Key> keys(total);
   if(total){
    emit_rays<<<(n+255)/256,256>>>(input.ptr,offsets.ptr,keys.ptr,n,r,lo,hi);check(cudaGetLastError());
    auto first=thrust::device_pointer_cast(keys.ptr);thrust::sort(first,first+total);auto last=thrust::unique(first,first+total);
    size_t unique=last-first;std::vector<Key> host(unique);check(cudaMemcpy(host.data(),keys.ptr,unique*sizeof(Key),cudaMemcpyDeviceToHost));free.insert(host.begin(),host.end());
   }
   check(cudaEventRecord(end));check(cudaEventSynchronize(end));float ms;check(cudaEventElapsedTime(&ms,begin,end));result.milliseconds+=ms;
  }catch(...){cudaEventDestroy(begin);cudaEventDestroy(end);throw;}
  cudaEventDestroy(begin);cudaEventDestroy(end);base+=n;
 }
 result.free.assign(free.begin(),free.end());std::sort(result.free.begin(),result.free.end());return result;
}
}
