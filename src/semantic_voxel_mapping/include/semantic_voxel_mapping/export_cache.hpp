#pragma once
#include "semantic_voxel_mapping/core.hpp"
#include <octomap/ColorOcTree.h>
#include <climits>
#include <memory>

// Derived exports only; probability and semantic history remain owned by Map.
// Apply changed cells instead of rebuilding millions of free octree leaves
// on every publish. Column counts preserve free/occupied/unknown projection.
class MapExportCache {
 std::unique_ptr<octomap::ColorOcTree> tree_;
 std::unordered_map<svm::Key,svm::Label> occupied_;
 std::unordered_map<uint64_t,size_t> columns_;
 bool inner_dirty_=false;
public:
 int minx=INT_MAX,miny=INT_MAX,maxx=INT_MIN,maxy=INT_MIN;
 explicit MapExportCache(double resolution):tree_(std::make_unique<octomap::ColorOcTree>(resolution)){}
 void update(const svm::Map& map,svm::Key key){
  const auto& cell=map.cells.at(key);auto label=map.label(cell);
  const auto p=svm::center(key,map.config.resolution);
  auto* node=tree_->setNodeValue(octomap::point3d(p.x,p.y,p.z),cell.odds,true);
  if(!node)throw std::runtime_error("Voxel outside OctoMap export extent");
  node->setColor((label.color>>16)&255,(label.color>>8)&255,label.color&255);inner_dirty_=true;
  int x=svm::coordinate(key,0),y=svm::coordinate(key,1);
  minx=std::min(minx,x);miny=std::min(miny,y);maxx=std::max(maxx,x);maxy=std::max(maxy,y);
  auto& count=columns_[(uint64_t(uint32_t(x))<<32)|uint32_t(y)];
  bool before=occupied_.count(key),after=map.occupied(cell);
  if(after&&!before)++count;
  if(before&&!after)--count;
  if(after)occupied_[key]=label;else occupied_.erase(key);
 }
 void rebuild(const svm::Map& map){
  *this=MapExportCache(map.config.resolution);
  for(const auto& entry:map.cells)update(map,entry.first);
 }
 octomap::ColorOcTree& tree(){
  if(inner_dirty_){
   // Preserve voted leaf colors: occupancy propagation must not average them.
   static_cast<octomap::OccupancyOcTreeBase<octomap::ColorOcTreeNode>&>(*tree_).updateInnerOccupancy();
   inner_dirty_=false;
  }
  return *tree_;
 }
 const auto& occupied()const{return occupied_;}
 const auto& columns()const{return columns_;}
};
