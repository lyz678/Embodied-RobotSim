#pragma once
#include "core.hpp"
#ifdef __CUDACC__
#define SVM_HD __host__ __device__
#else
#define SVM_HD
#endif
namespace svm {
// Half-open voxels. At exact edge/corner crossings advance all tied axes;
// merely touching a voxel at a zero-length boundary does not clear it.
SVM_HD inline unsigned trace(const Ray& ray,double r,double zmin,double zmax,Key* output){
 int c[3]={int(floor(ray.origin.x/r)),int(floor(ray.origin.y/r)),int(floor(ray.origin.z/r))};
 int end[3]={int(floor(ray.end.x/r)),int(floor(ray.end.y/r)),int(floor(ray.end.z/r))};
 double a[3]={ray.origin.x,ray.origin.y,ray.origin.z},b[3]={ray.end.x,ray.end.y,ray.end.z},t[3],dt[3];int step[3];
 for(int i=0;i<3;++i){double d=b[i]-a[i];step[i]=d>0?1:d<0?-1:0;dt[i]=step[i]?r/fabs(d):INFINITY;t[i]=step[i]?((c[i]+(step[i]>0?1:0))*r-a[i])/d:INFINITY;}
 unsigned count=0;int limit=abs(end[0]-c[0])+abs(end[1]-c[1])+abs(end[2]-c[2])+2;
 for(int iteration=0;iteration<limit;++iteration){
  bool final=c[0]==end[0]&&c[1]==end[1]&&c[2]==end[2];double z=(c[2]+.5)*r;
  double next=fmin(t[0],fmin(t[1],t[2]));
  if((final||next>0)&&!(final&&ray.hit)&&z>=zmin&&z<=zmax){
   if(output)output[count]=Key(c[0]+BIAS)|(Key(c[1]+BIAS)<<21)|(Key(c[2]+BIAS)<<42);++count;
  }
  if(final)break;
  // Mixed-sign rays ending exactly on a corner must not step past the
  // endpoint on negative axes while entering its voxel on positive axes.
  if(next>=1.-1e-12){
   double end_z=(end[2]+.5)*r;
   if(!ray.hit&&end_z>=zmin&&end_z<=zmax){
    if(output)output[count]=Key(end[0]+BIAS)|(Key(end[1]+BIAS)<<21)|(Key(end[2]+BIAS)<<42);++count;
   }
   break;
  }
  for(int i=0;i<3;++i)if(t[i]<=next+1e-12){c[i]+=step[i];t[i]+=dt[i];}
 }
 return count;
}
}
#undef SVM_HD
