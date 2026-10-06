#include "semantic_voxel_mapping/core.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <random>
using namespace svm;
int main(int argc,char** argv){try{
 require_cuda(0);std::vector<Ray> rays;std::mt19937 random(42);std::uniform_real_distribution<double> angle(-1.,1.),range(.2,8.);
 for(int i=0;i<40000;++i){double x=angle(random),y=angle(random),z=angle(random);double norm=std::sqrt(x*x+y*y+z*z),d=range(random);Vec origin{-.13,.21,.78};
 rays.push_back({origin,{origin.x+x*d/norm,origin.y+y*d/norm,origin.z+z*d/norm},bool(i%3)});}
 // Boundary, negative, zero length, axis-aligned and exact corner cases.
 for(auto end:std::vector<Vec>{{-3.,0.,.5},{0.,-3.,.5},{1.,1.,1.},{0.,0.,0.},{.1,.2,.3}})rays.push_back({{0,0,0},end,true});
 rays.push_back({{.5,.5,.5},{1.,0.,.5},true});
 rays.push_back({{.5,.5,.5},{1.,0.,.5},false});
 if(argc>1){rays.clear();std::ifstream in(argv[1],std::ios::binary);Ray ray;while(in.read(reinterpret_cast<char*>(&ray),sizeof(ray)))rays.push_back(ray);if(rays.empty())throw std::runtime_error("No recorded rays");}
 auto start=std::chrono::steady_clock::now();auto cpu=raycast_reference(rays,.1,.1,2.5);double cpu_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
 auto gpu=raycast_cuda(rays,.1,.1,2.5,512ull*1024*1024);
 if(cpu!=gpu.free)throw std::runtime_error("CUDA/CPU raycast output mismatch");
 std::cout<<"{\"rays\":"<<rays.size()<<",\"free_voxels\":"<<cpu.size()<<",\"cpu_ms\":"<<cpu_ms<<",\"gpu_ms\":"<<gpu.milliseconds<<",\"scratch_bytes_upper_bound\":"<<gpu.peak_scratch_bytes<<",\"equal\":true}\n";
 return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
