#pragma once
#include "trace.hpp"
namespace svm {
// Only label the measured endpoint when it is the first occupied surface.
// A foreground cell blocks the semantic ray; free/unknown space receives no vote.
inline bool visible_endpoint(const Map& map,Ray ray,double min_range,double zmin,double zmax){
 ray.hit=false;const auto target=key(ray.end,map.config.resolution);
 auto count=trace(ray,map.config.resolution,zmin,zmax,nullptr);std::vector<Key> path(count);
 trace(ray,map.config.resolution,zmin,zmax,path.data());
 for(auto k:path){
  auto p=center(k,map.config.resolution);
  if(std::sqrt(std::pow(p.x-ray.origin.x,2)+std::pow(p.y-ray.origin.y,2)+std::pow(p.z-ray.origin.z,2))<min_range)continue;
  auto it=map.cells.find(k);
  if(it!=map.cells.end()&&map.occupied(it->second))return k==target;
 }
 return false;
}
}
