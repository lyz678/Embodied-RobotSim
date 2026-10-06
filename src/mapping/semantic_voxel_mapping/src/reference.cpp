#include "semantic_voxel_mapping/trace.hpp"
namespace svm {
std::vector<Key> raycast_reference(const std::vector<Ray>& rays,double resolution,double zmin,double zmax){
 std::unordered_set<Key> result;
 for(auto& ray:rays){auto n=trace(ray,resolution,zmin,zmax,nullptr);std::vector<Key> keys(n);trace(ray,resolution,zmin,zmax,keys.data());result.insert(keys.begin(),keys.end());}
 std::vector<Key> sorted(result.begin(),result.end());std::sort(sorted.begin(),sorted.end());return sorted;
}
}
