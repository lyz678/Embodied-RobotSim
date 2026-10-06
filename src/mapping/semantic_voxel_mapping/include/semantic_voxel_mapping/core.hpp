#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace svm {
using Key=uint64_t; using Color=uint32_t;
constexpr Color UNKNOWN=0xffffff;
constexpr int BIAS=1<<20;
struct Vec { double x,y,z; };
struct Ray { Vec origin,end; bool hit; };
inline int coordinate(Key k,int axis) { return int((k>>(21*axis))&0x1fffff)-BIAS; }
inline Key key(int x,int y,int z) {
 for(int v:{x,y,z}) if(v < -BIAS || v>=BIAS) throw std::out_of_range("Voxel key outside representable range");
 return Key(x+BIAS)|(Key(y+BIAS)<<21)|(Key(z+BIAS)<<42);
}
inline Key key(Vec p,double r) {return key(int(std::floor(p.x/r)),int(std::floor(p.y/r)),int(std::floor(p.z/r)));}
inline Vec center(Key k,double r) {return {(coordinate(k,0)+.5)*r,(coordinate(k,1)+.5)*r,(coordinate(k,2)+.5)*r};}
inline double logit(double p) {return std::log(p/(1-p));}
struct Config {
 double resolution=.1, hit=.7, miss=.4, clamp_min=.12, clamp_max=.97, occupied=.5;
 unsigned window=30,min_observations=3; double majority=.6;
 size_t max_voxels=2000000;
 void validate() const {
  if(!std::isfinite(resolution)||resolution<=0||!std::isfinite(majority)||majority<=.5||majority>1||
   !(clamp_min>0&&clamp_min<.5&&clamp_min<occupied&&occupied<clamp_max&&clamp_max>.5&&clamp_max<1)||
   !(hit>.5&&hit<1&&miss>0&&miss<.5)||window<min_observations||min_observations<1||window>10000||max_voxels<1)
   throw std::invalid_argument("Invalid occupancy or semantic configuration");
 }
};
struct Cell {double odds=0;std::vector<Color> history;};
struct Frame {
 std::unordered_set<Key> free,hits;
 std::unordered_map<Key,std::unordered_map<Color,unsigned>> votes;
};
struct Label {Color color=UNKNOWN;double fraction=0;unsigned observations=0;};
class Map {
 public:
 Config config;std::unordered_map<Key,Cell> cells;
 explicit Map(Config c={}):config(c){config.validate();}
 bool occupied(const Cell& c)const{return c.odds>logit(config.occupied);}
 Label label(const Cell& c)const{
  Label result; result.observations=c.history.size();if(c.history.empty())return result;
  std::unordered_map<Color,unsigned> count;for(auto color:c.history)++count[color];
  unsigned best=0;Color color=UNKNOWN;bool tie=false;
  for(auto entry:count){if(entry.second>best){best=entry.second;color=entry.first;tie=false;}else if(entry.second==best)tie=true;}
  result.fraction=double(best)/c.history.size();
  if(occupied(c)&&!tie&&c.history.size()>=config.min_observations&&result.fraction>=config.majority)result.color=color;
  return result;
 }
 std::vector<Key> commit(const Frame& frame){
  size_t additions=0;for(auto k:frame.hits)if(!cells.count(k))++additions;
  for(auto k:frame.free)if(!frame.hits.count(k)&&!cells.count(k))++additions;
  if(additions>config.max_voxels-cells.size())throw std::runtime_error("Map voxel budget exceeded; frame rejected");
  // Allocate/copy all modified nodes before changing canonical state. Node
  // handles are merged after bucket reservation, so allocation failure cannot
  // leave half a frame's evidence committed.
  std::unordered_map<Key,Cell> changes;
  changes.reserve(frame.hits.size()+frame.free.size());
  auto cell_copy=[&](Key k)->Cell& {
   auto current=cells.find(k);
   return changes.emplace(k,current==cells.end()?Cell{}:current->second).first->second;
  };
  const double lower=logit(config.clamp_min),upper=logit(config.clamp_max);
  for(auto k:frame.free)if(!frame.hits.count(k)){
   auto current=cells.find(k);
   if(current!=cells.end()&&current->second.odds<=lower&&current->second.history.empty())continue;
   auto& c=cell_copy(k);c.odds=std::max(lower,c.odds+logit(config.miss));if(!occupied(c))c.history.clear();
  }
  for(auto k:frame.hits){
   auto current=cells.find(k);
   if(current!=cells.end()&&current->second.odds>=upper&&!frame.votes.count(k))continue;
   auto& c=cell_copy(k);c.odds=std::min(upper,c.odds+logit(config.hit));
   auto it=frame.votes.find(k);if(it==frame.votes.end())continue;
   unsigned best=0;Color color=UNKNOWN;bool tie=false;
   for(auto entry:it->second){if(entry.first==UNKNOWN)continue;
    if(entry.second>best){best=entry.second;color=entry.first;tie=false;}else if(entry.second==best)tie=true;
   }
   if(!best||tie)continue;
   if(c.history.size()==config.window)c.history.erase(c.history.begin());c.history.push_back(color);
  }
  std::vector<Key> changed;changed.reserve(changes.size());
  for(const auto& entry:changes)changed.push_back(entry.first);
  cells.reserve(cells.size()+additions);
  for(auto& entry:changes){auto old=cells.find(entry.first);if(old!=cells.end())std::swap(old->second,entry.second);}
  cells.merge(changes);
  return changed;
 }
 template<class T>static void write(std::ostream& s,T value){s.write(reinterpret_cast<const char*>(&value),sizeof(value));}
 template<class T>static T read(std::istream& s){T v{};s.read(reinterpret_cast<char*>(&v),sizeof(v));if(!s)throw std::runtime_error("Truncated map file");return v;}
 void save(std::ostream& out,const std::string& signature)const{
  out.write("SVMAP001",8);write<uint32_t>(out,signature.size());out.write(signature.data(),signature.size());write<uint64_t>(out,cells.size());
  for(auto& entry:cells){write(out,entry.first);write(out,entry.second.odds);write<uint32_t>(out,entry.second.history.size());for(auto c:entry.second.history)write(out,c);}
  if(!out)throw std::runtime_error("Map write failed");
 }
 void load(std::istream& in,const std::string& signature,const std::unordered_set<Color>& palette){
  char magic[8];in.read(magic,8);if(!in||std::memcmp(magic,"SVMAP001",8))throw std::runtime_error("Invalid map file");
  auto length=read<uint32_t>(in);if(length>1048576)throw std::runtime_error("Invalid metadata length");
  std::string actual(length,'\0');in.read(actual.data(),length);if(!in||actual!=signature)throw std::runtime_error("Incompatible map frame/configuration/palette");
  auto count=read<uint64_t>(in);if(count>config.max_voxels)throw std::runtime_error("Map exceeds voxel budget");
  std::unordered_map<Key,Cell> loaded;
  for(uint64_t i=0;i<count;++i){auto k=read<Key>(in);Cell c;c.odds=read<double>(in);auto n=read<uint32_t>(in);
   if(k>>63||!std::isfinite(c.odds)||c.odds<logit(config.clamp_min)-1e-8||c.odds>logit(config.clamp_max)+1e-8||n>config.window)throw std::runtime_error("Invalid saved voxel");
   for(unsigned j=0;j<n;++j){auto color=read<Color>(in);if(!palette.count(color))throw std::runtime_error("Invalid saved label");c.history.push_back(color);}
   if(!loaded.emplace(k,std::move(c)).second)throw std::runtime_error("Duplicate saved voxel");
  }
  if(in.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Unexpected map trailer");cells.swap(loaded);
 }
};
struct GpuResult {std::vector<Key> free;double milliseconds=0;size_t peak_scratch_bytes=0;};
void require_cuda(int device);
GpuResult raycast_cuda(const std::vector<Ray>& rays,double resolution,double zmin,double zmax,size_t budget);
std::vector<Key> raycast_reference(const std::vector<Ray>& rays,double resolution,double zmin,double zmax);
}
