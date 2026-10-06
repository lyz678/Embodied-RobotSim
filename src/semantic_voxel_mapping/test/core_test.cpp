#include "semantic_voxel_mapping/core.hpp"
#include <iostream>
#include <sstream>
using namespace svm;
static void require(bool test,const char* message){if(!test)throw std::runtime_error(message);}
int main(){try{
 const Color red=0xff0000,blue=0x0000ff;auto k=key(-2,3,4);
 require(coordinate(k,0)==-2&&coordinate(k,1)==3&&coordinate(k,2)==4,"Signed voxel keys");
 auto rays=std::vector<Ray>{{{.5,.5,.5},{3.5,.5,.5},true}};auto free=raycast_reference(rays,1,-10,10);
 require(free==std::vector<Key>({key(0,0,0),key(1,0,0),key(2,0,0)}),"Endpoint excluded from free ray");
 rays[0].hit=false;free=raycast_reference(rays,1,-10,10);require(free.size()==4,"Truncated endpoint is free");
 rays={{{.5,.5,.5},{-2.5,-2.5,-2.5},true}};free=raycast_reference(rays,1,-10,10);
 require(free.size()==3&&std::find(free.begin(),free.end(),key(-1,-1,-1))!=free.end(),"Diagonal ties and negative coordinates");
 require(raycast_reference(rays,1,10,20).empty(),"Height clipping");
 rays={{{.5,.5,.5},{1.,0.,.5},true}};free=raycast_reference(rays,1,-10,10);
 require(free==std::vector<Key>({key(0,0,0)}),"Mixed-sign corner endpoint cannot overshoot ray");
 rays[0].hit=false;free=raycast_reference(rays,1,-10,10);
 require(free==std::vector<Key>({key(0,0,0),key(1,0,0)}),"Mixed-sign truncated endpoint is included once");
 rays={{{0.,.5,.5},{-3.,.5,.5},true}};free=raycast_reference(rays,1,-10,10);
 require(free==std::vector<Key>({key(-2,0,0),key(-1,0,0)}),"Zero-length origin boundary contact must not clear origin voxel");
 Map map;Frame f;f.hits.insert(k);f.free.insert(k);f.votes[k][red]=100000;map.commit(f);
 require(std::abs(map.cells.at(k).odds-logit(.7))<1e-10,"Hit takes precedence; duplicate pixels cannot multiply hit");
 require(map.label(map.cells.at(k)).color==UNKNOWN&&map.cells.at(k).history.size()==1,"Dense single frame has one vote and cannot confirm semantic label");
 map.commit(f);map.commit(f);require(map.label(map.cells.at(k)).color==red,"Three observations confirm class");
 Frame bad;bad.hits.insert(k);bad.votes[k][blue]=999999;map.commit(bad);
 require(map.label(map.cells.at(k)).color==red,"One dense wrong frame cannot overwhelm temporal majority");
 Frame tie;tie.hits.insert(k);tie.votes[k][red]=10;tie.votes[k][blue]=10;auto before=map.cells.at(k).history.size();map.commit(tie);
 require(map.cells.at(k).history.size()==before,"Per-frame ties abstain");
 Frame unknown;unknown.hits.insert(k);unknown.votes[k][UNKNOWN]=999;map.commit(unknown);
 require(map.cells.at(k).history.size()==before,"Unknown observations abstain");
 for(int i=0;i<100;++i)map.commit(f);require(map.cells.at(k).history.size()==30&&map.cells.at(k).odds<=logit(.97),"Bounded history and occupancy clamping");
 for(int i=0;i<30;++i)map.commit(bad);require(map.label(map.cells.at(k)).color==blue,"Recent observations replace early mistakes");
 Frame clear;clear.free.insert(k);for(int i=0;i<100;++i)map.commit(clear);
 require(!map.occupied(map.cells.at(k))&&map.cells.at(k).history.empty()&&map.cells.at(k).odds>=logit(.12),"Free evidence clears occupancy and semantic history");
 map.commit(f);require(map.label(map.cells.at(k)).color==UNKNOWN,"Reappearing object needs fresh confirmation");
 std::stringstream saved;map.save(saved,"frame=odom,palette=v1");Map restored;restored.load(saved,"frame=odom,palette=v1",{red,blue});
 require(restored.cells.at(k).odds==map.cells.at(k).odds&&restored.cells.at(k).history==map.cells.at(k).history,"Persistence preserves probabilities and complete vote window");
 std::stringstream wrong;map.save(wrong,"frame=odom,palette=v1");bool rejected=false;try{restored.load(wrong,"frame=map,palette=v1",{red,blue});}catch(...){rejected=true;}
 require(rejected&&restored.cells.size()==map.cells.size(),"Incompatible load is atomic");
 Config limited;limited.max_voxels=1;Map small(limited);small.commit(f);Frame extra;extra.free.insert(key(1,1,1));extra.hits.insert(k);double original=small.cells.at(k).odds;rejected=false;
 try{small.commit(extra);}catch(...){rejected=true;}require(rejected&&small.cells.at(k).odds==original,"Voxel budget rejects whole frame");
 std::cout<<"PASS: occupancy, temporal voting, clearing, DDA geometry, persistence and transaction limits\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
