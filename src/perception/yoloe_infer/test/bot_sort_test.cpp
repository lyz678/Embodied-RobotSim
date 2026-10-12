#include "bot_sort.hpp"
#include "neck_reid.hpp"
#include "lidar_localization.hpp"
#include <iostream>
using namespace yoloe_infer;
void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
TrackObservation observation(float x,double world_x,std::vector<float> feature={1,0},double score=.9) {
    TrackObservation o;o.box={x,10,20,30};o.position={world_x,0,5};o.has_position=true;o.score=score;o.appearance=feature;return o;
}
int main() {
    try {
        TrackerConfig cfg;cfg.min_hits=1;
        BotSort3D tracker(cfg);
        std::vector<TrackObservation> frame={observation(0,0),observation(30,1,{0,1})};
        tracker.update(frame,0);int a=frame[0].track_id,b=frame[1].track_id;
        require(a>0&&b>a,"Unique birth IDs");
        frame={observation(15,.9,{0,1}),observation(15,.1)};
        tracker.update(frame,.1);
        require(frame[0].track_id==b&&frame[1].track_id==a,"Crossing tracks retain identity using geometry and appearance");
        frame={observation(15,.2,{1,0},.2)};tracker.update(frame,.2);
        require(frame[0].track_id==a,"Low confidence second stage continues track");
        frame={};tracker.update(frame,.3);
        frame={observation(15,.3)};tracker.update(frame,.4);
        require(frame[0].track_id==a,"Short occlusion reactivation");
        require(tracker.tracks()[0].x[11]>0,"World velocity is learned");
        for(const auto& t:tracker.tracks()) {
            require(t.covariance.allFinite(),"Finite covariance");
            require((t.covariance-t.covariance.transpose()).norm()<1e-8,"Symmetric covariance");
            require(t.covariance.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff()>-1e-8,"Positive semidefinite covariance");
        }
        frame={observation(15,20)};tracker.update(frame,.5);
        require(frame[0].track_id!=a,"Spatial gate rejects impossible jump");
        int old=frame[0].track_id;frame={observation(15,20)};tracker.update(frame,4);
        require(frame[0].track_id!=old,"Expired track gets a new ID");
        old=frame[0].track_id;tracker.update(frame,1);
        require(frame[0].track_id!=old,"Clock rewind clears trajectories without recycling IDs");
        BotSort3D low(cfg);frame={observation(0,0,{1,0},.2)};low.update(frame,0);
        require(low.tracks().empty(),"Low detections cannot create tracks");
        BotSort3D image_only(cfg);frame={observation(0,0)};frame[0].has_position=false;image_only.update(frame,0);old=frame[0].track_id;
        frame={observation(1,0)};frame[0].has_position=false;image_only.update(frame,.1);
        require(frame[0].track_id==old&&!image_only.tracks()[0].has_position,"2D fallback never invents geometry");
        TrackerConfig jitter_cfg=cfg;jitter_cfg.tolerate_class_jitter=true;
        BotSort3D jitter(jitter_cfg);
        frame={observation(0,0)};jitter.update(frame,0);old=frame[0].track_id;
        frame={observation(0,0)};frame[0].class_id=4;jitter.update(frame,.1);
        require(frame[0].track_id==old&&frame[0].class_id==4&&jitter.tracks()[0].class_id==0,
                "Identity recovery preserves raw class; historical class stays on track only");
        frame={observation(0,0)};jitter.update(frame,.2);
        for(double stamp:{.3,.4,.5}) {
            frame={observation(0,0)};frame[0].class_id=4;jitter.update(frame,stamp);
            require(frame[0].track_id==old,"Sustained label correction keeps identity");
        }
        require(frame[0].class_id==4&&jitter.tracks()[0].class_id==4,"Repeated evidence corrects track class without relabeling observations");
        frame={observation(0,0,{0,1})};frame[0].class_id=6;jitter.update(frame,.6);
        require(frame[0].track_id!=old,"Different appearance cannot bridge class mismatch");
        BotSort3D strict(cfg);frame={observation(0,0)};strict.update(frame,0);old=frame[0].track_id;
        frame={observation(0,0)};frame[0].class_id=4;strict.update(frame,.1);
        require(frame[0].track_id!=old,"Strict class gating remains the default");
        TrackerConfig manipulation_cfg=cfg;manipulation_cfg.high=.2;manipulation_cfg.birth=.25;manipulation_cfg.min_hits=3;
        BotSort3D bottle(manipulation_cfg);frame={observation(0,0,{1,0},.26)};bottle.update(frame,0);
        require(frame[0].track_id<0,"Birth waits for confirmation");
        frame={observation(0,0,{1,0},.23)};bottle.update(frame,.1);
        frame={observation(0,0,{1,0},.22)};bottle.update(frame,.2);old=frame[0].track_id;
        require(old>0,"Observed bottle confidence can confirm a track");
        frame={observation(0,0,{1,0},.15)};bottle.update(frame,.3);
        require(frame[0].track_id==old,"Confirmed bottle survives confidence dip");
        TrackerConfig ego_cfg=cfg;ego_cfg.match=.4;
        BotSort3D stale(cfg);frame={observation(0,0)};stale.update(frame,0);old=frame[0].track_id;
        for(double stamp:{.5,1.,1.5,2.1}) {frame={observation(0,0)};frame[0].has_position=false;stale.update(frame,stamp);}
        require(!stale.tracks()[0].has_position,"Old 3D measurement expires while 2D tracking continues");
        frame={observation(0,20)};stale.update(frame,2.2);
        require(frame[0].track_id==old&&stale.tracks()[0].has_position,"Reacquire geometry on existing 2D track");
        require(stale.tracks()[0].covariance.selfadjointView<Eigen::Lower>().eigenvalues().minCoeff()>-1e-8,"Reacquired covariance remains positive semidefinite");
        BotSort3D ego(ego_cfg);frame={observation(40,0)};ego.update(frame,0);old=frame[0].track_id;
        Eigen::Matrix4d pose=Eigen::Matrix4d::Identity();pose(0,3)=-2;
        Eigen::Matrix3d k;k<<100,0,50,0,100,25,0,0,1;
        frame={observation(0,0)};ego.update(frame,.1,&pose,&k);
        require(frame[0].track_id==old,"FAST-LIO pose compensation survives zero box overlap");
        auto assignment=assign_tracks({{.1,.2},{.11,.69}},.7);
        require(assignment[0]==1&&assignment[1]==0,"Global assignment beats greedy association");
        assignment=assign_tracks({{.1,9},{.2,9}},.7);
        require((assignment[0]<0)!=(assignment[1]<0),"Forbidden edges leave rows unmatched");
        float tensor[]={1,1,2,2, 0,0,2,2};
        auto feature=pool_neck(tensor,2,2,2,{0,0,2,1},1,0,0,2,2);
        require(feature.size()==2&&std::abs(feature[0]-1)<1e-6&&feature[1]==0,"ROI neck pooling and normalization");
        feature=pool_neck(tensor,2,2,2,{-20,-20,1,1},1,0,0,2,2);
        require(feature.empty(),"Out of image ROI has no appearance");
        std::vector<ProjectedLidarPoint> points;
        for(int i=0;i<6;++i) {points.push_back({i,1,2,{double(i)*.01,0,2}});points.push_back({i,1,8,{0,0,8}});}
        points.push_back({8,1,.2,{100,0,.2}}); // isolated foreground outlier
        Eigen::Vector3d position;
        require(lidar_position({0,0,10,10},{},points,10,10,position)&&position.z()==2,"Reject occluded background and isolated returns");
        require(!lidar_position({0,0,10,10},cv::Mat::zeros(10,10,CV_8U),points,10,10,position),"Mask excludes background");
        require(!lidar_position({0,0,10,10},{},points,10,10,position,20),"Sparse lidar produces missing position");
        bool rejected=false;try{cfg.spatial_gate=0;BotSort3D invalid(cfg);}catch(const std::invalid_argument&){rejected=true;}
        require(rejected,"Reject invalid configuration");
        std::cout<<"PASS: identity, two-stage association, occlusion, expiration, rewind, covariance, ego compensation, Hungarian, Neck ROI, LiDAR foreground\n";
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
