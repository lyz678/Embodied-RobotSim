#include "semantic_voxel_mapping/export_cache.hpp"
#include <iostream>
#include <sstream>
void require(bool value,const char* text){if(!value)throw std::runtime_error(text);}
void compare(svm::Map& map,MapExportCache& cache){
 octomap::ColorOcTree reference(map.config.resolution);
 std::unordered_map<uint64_t,size_t> columns;
 size_t count=0;
 for(const auto& e:map.cells){
  auto p=svm::center(e.first,map.config.resolution);auto label=map.label(e.second);
  auto* leaf=reference.updateNode(octomap::point3d(p.x,p.y,p.z),map.occupied(e.second),true);
  leaf->setLogOdds(e.second.odds);leaf->setColor((label.color>>16)&255,(label.color>>8)&255,label.color&255);
  int x=svm::coordinate(e.first,0),y=svm::coordinate(e.first,1);columns[(uint64_t(uint32_t(x))<<32)|uint32_t(y)]+=map.occupied(e.second);
  count+=map.occupied(e.second);
 }
 require(cache.columns()==columns,"Projection must clear a column only when all its occupied heights clear");
 require(cache.occupied().size()==count,"Occupied cache must match canonical evidence");
 auto& tree=cache.tree();
 for(const auto& e:map.cells){
  auto p=svm::center(e.first,map.config.resolution);auto* a=tree.search(p.x,p.y,p.z);auto* b=reference.search(p.x,p.y,p.z);
  require(a&&b&&a->getLogOdds()==b->getLogOdds()&&a->getColor()==b->getColor(),"Incremental export must preserve full free/occupied probabilities and voted colors");
 }
}
int main(){try{
 svm::Map map;MapExportCache cache(map.config.resolution);auto k=svm::key(-2,3,4),k2=svm::key(-2,3,5);
 auto commit=[&](const svm::Frame& f){auto changed=map.commit(f);for(auto key:changed)cache.update(map,key);compare(map,cache);return changed;};
 svm::Frame hit;hit.hits={k,k2};hit.votes[k][0xff0000]=10;
 for(int i=0;i<20;++i)commit(hit);
 svm::Frame miss;miss.free={k};for(int i=0;i<30;++i)commit(miss);
 require(cache.occupied().count(k)==0&&cache.occupied().count(k2)==1,"Moved object leaves no occupied ghost, other height remains");
 require(commit(miss).empty(),"Clamped free cells must not trigger redundant copies or export work");
 svm::Frame both;both.free={k};both.hits={k};both.votes[k][0x0000ff]=10;for(int i=0;i<15;++i)commit(both);
 require(cache.occupied().at(k).color==0x0000ff,"Reappearing surface needs fresh semantic votes");
 std::stringstream saved;map.save(saved,"test");svm::Map loaded;loaded.load(saved,"test",{0xff0000,0x0000ff});cache.rebuild(loaded);compare(loaded,cache);
 loaded.cells.clear();cache.rebuild(loaded);compare(loaded,cache);require(cache.tree().size()==0,"Clear must remove cached leaves");
 std::cout<<"PASS: incremental exports, occupied columns, saturation, semantic changes, load and clear\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
