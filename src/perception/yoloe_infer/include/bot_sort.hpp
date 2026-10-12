#pragma once
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace yoloe_infer {
struct TrackObservation {
    cv::Rect2f box;
    int class_id = 0;
    double score = 0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    bool has_position = false;
    std::vector<float> appearance;
    int track_id = -1;
};
struct TrackerConfig {
    double low = .1, high = .5, birth = .6;
    double max_age = 2., match = .7, spatial_gate = 2.;
    double iou_weight = .3, appearance_weight = .3, spatial_weight = .4;
    double measurement_variance = .04, acceleration_variance = 1.;
    int min_hits = 2;
    bool tolerate_class_jitter = false;
    int class_switch_hits = 3;
    void validate() const {
        for (double v : {low, high, birth, max_age, match, spatial_gate, iou_weight,
                        appearance_weight, spatial_weight, measurement_variance, acceleration_variance})
            if (!std::isfinite(v) || v < 0) throw std::invalid_argument("Invalid tracking parameter");
        if (!(low < high && high <= birth && birth <= 1 && max_age > 0 &&
              match > 0 && match < 1 && spatial_gate > 0 && measurement_variance > 0 &&
              min_hits >= 1 && class_switch_hits >= 2 && iou_weight > 0)) throw std::invalid_argument("Invalid tracking thresholds");
    }
};
struct Track {
    int id = 0, class_id = 0, hits = 0;
    int pending_class = -1, pending_class_hits = 0;
    double last_seen = 0, last_position = 0, score = 0;
    bool has_position = false, updated = false;
    // Joint constant velocity state: [cx,cy,w,h,X,Y,Z, vcx,vcy,vw,vh,VX,VY,VZ].
    Eigen::Matrix<double,14,1> x = Eigen::Matrix<double,14,1>::Zero();
    Eigen::Matrix<double,14,14> covariance = Eigen::Matrix<double,14,14>::Identity();
    std::vector<float> appearance;
    cv::Rect2f box() const {
        return {float(x[0]-x[2]/2), float(x[1]-x[3]/2), float(std::max(1.,x[2])), float(std::max(1.,x[3]))};
    }
};
inline bool normalize_feature(std::vector<float>& f) {
    double n = 0;
    for (float v : f) { if (!std::isfinite(v)) { f.clear(); return false; } n += v*v; }
    if (n < 1e-12) { f.clear(); return false; }
    for (auto& v : f) v /= std::sqrt(n);
    return true;
}
// Rectangular Hungarian assignment with explicit unmatched columns. Forbidden
// edges never consume a detection, unlike assigning then filtering a dense LAP.
inline std::vector<int> assign_tracks(const std::vector<std::vector<double>>& cost, double limit) {
    int n = cost.size();
    if (!n) return {};
    int real = cost[0].size(), m = real+n;
    std::vector<double> u(n+1), v(m+1);
    std::vector<int> p(m+1), way(m+1), result(n,-1);
    for (int i=1; i<=n; ++i) {
        p[0]=i; int j0=0;
        std::vector<double> minimum(m+1, 1e9); std::vector<bool> used(m+1,false);
        do {
            used[j0]=true; int i0=p[j0], j1=0; double delta=1e9;
            for (int j=1; j<=m; ++j) if (!used[j]) {
                double c = j<=real ? cost[i0-1][j-1] : limit;
                if (c >= limit && j<=real) c=1e6;
                double cur=c-u[i0]-v[j];
                if (cur<minimum[j]) {minimum[j]=cur; way[j]=j0;}
                if (minimum[j]<delta) {delta=minimum[j]; j1=j;}
            }
            for (int j=0;j<=m;++j) if (used[j]) {u[p[j]]+=delta;v[j]-=delta;} else minimum[j]-=delta;
            j0=j1;
        } while (p[j0]);
        do {int j1=way[j0];p[j0]=p[j1];j0=j1;} while(j0);
    }
    for(int j=1;j<=real;++j) if(p[j]) result[p[j]-1]=j-1;
    return result;
}
class BotSort3D {
public:
    explicit BotSort3D(TrackerConfig config = {}) : config_(config) {config_.validate();}
    const std::vector<Track>& tracks() const {return tracks_;}
    void clear() {tracks_.clear(); previous_=std::numeric_limits<double>::quiet_NaN();}
    void update(std::vector<TrackObservation>& observations, double stamp,
                const Eigen::Matrix4d* world_to_camera = nullptr, const Eigen::Matrix3d* intrinsics = nullptr) {
        if(!std::isfinite(stamp)) throw std::invalid_argument("Nonfinite tracking timestamp");
        if(std::isfinite(previous_) && stamp==previous_) return;
        if(std::isfinite(previous_) && stamp<previous_) clear();
        double dt=std::isfinite(previous_) ? stamp-previous_ : 0; previous_=stamp;
        tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track& t){return stamp-t.last_seen>config_.max_age;}),tracks_.end());
        for(auto& t:tracks_) {
            t.updated=false;
            if(t.has_position && stamp-t.last_position>config_.max_age) t.has_position=false;
            Eigen::Matrix<double,14,14> f=Eigen::Matrix<double,14,14>::Identity(), q=Eigen::Matrix<double,14,14>::Zero();
            for(int k=0;k<7;++k) {
                f(k,k+7)=dt;
                double a=config_.acceleration_variance*(k<4?100.:1.);
                q(k,k)=a*std::pow(dt,4)/4; q(k,k+7)=q(k+7,k)=a*std::pow(dt,3)/2; q(k+7,k+7)=a*dt*dt;
            }
            t.x=f*t.x; t.covariance=f*t.covariance*f.transpose()+q;
            // FAST-LIO ego compensation: project the predicted world centroid
            // into the current camera rather than treating camera motion as target motion.
            if(t.has_position && world_to_camera && intrinsics) {
                Eigen::Vector4d p; p<<t.x.segment<3>(4),1;
                Eigen::Vector3d camera=((*world_to_camera)*p).head<3>();
                if(camera.z()>.1) {
                    Eigen::Vector3d uv=(*intrinsics)*camera;
                    t.x[0]=uv.x()/uv.z(); t.x[1]=uv.y()/uv.z();
                }
            }
        }
        std::vector<int> high, low;
        for(size_t i=0;i<observations.size();++i) {
            auto& o=observations[i]; o.track_id=-1; normalize_feature(o.appearance);
            if(!std::isfinite(o.score)||o.box.width<=0||o.box.height<=0||
               !std::isfinite(o.box.x)||!std::isfinite(o.box.y)||!std::isfinite(o.box.width)||!std::isfinite(o.box.height)) continue;
            if(o.has_position&&!o.position.allFinite()) o.has_position=false;
            if(o.score>=config_.high) high.push_back(i); else if(o.score>=config_.low) low.push_back(i);
        }
        std::vector<bool> consumed(observations.size(),false);
        associate(observations,high,consumed,stamp,false);
        associate(observations,low,consumed,stamp,true);
        tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[&](const Track& t){
            return t.hits<config_.min_hits && !t.updated;
        }),tracks_.end());
        for(int i:high) if(!consumed[i]&&observations[i].score>=config_.birth) {
            Track t; t.id=next_id_++; t.class_id=observations[i].class_id;
            t.covariance.diagonal().tail<7>().setConstant(10.);
            correct(t,observations[i],stamp); tracks_.push_back(t);
        }
    }
private:
    double cost(const Track& t,const TrackObservation& o,bool second) const {
        auto b=t.box(); double inter=(b&o.box).area(); double iou=inter/(b.area()+o.box.area()-inter+1e-9);
        const bool class_mismatch=t.class_id!=o.class_id;
        if(class_mismatch) {
            // Only near-identical boxes with strong appearance may bridge a
            // label flip. Class disagreement alone must not create a new ID.
            if(!config_.tolerate_class_jitter || iou<.7 || t.appearance.empty() ||
               t.appearance.size()!=o.appearance.size()) return 1e6;
            double dot=std::inner_product(t.appearance.begin(),t.appearance.end(),o.appearance.begin(),0.);
            if(dot<.85) return 1e6;
        }
        double total=config_.iou_weight*(1-iou), weight=config_.iou_weight;
        bool spatial=t.has_position&&o.has_position;
        if(spatial) {
            Eigen::Vector3d residual=o.position-t.x.segment<3>(4);
            Eigen::Matrix3d s=t.covariance.block<3,3>(4,4)+Eigen::Matrix3d::Identity()*config_.measurement_variance;
            if(residual.norm()>config_.spatial_gate || residual.dot(s.ldlt().solve(residual))>16.27) return 1e6;
            total+=config_.spatial_weight*residual.norm()/config_.spatial_gate;weight+=config_.spatial_weight;
        }
        // Without metric geometry, appearance cannot jump between distant boxes.
        if(!spatial&&iou<.05) return 1e6;
        if(!second&&!t.appearance.empty()&&t.appearance.size()==o.appearance.size()) {
            double dot=std::inner_product(t.appearance.begin(),t.appearance.end(),o.appearance.begin(),0.);
            double distance=std::clamp(1-dot,0.,2.)/2;
            if(distance>.5) return 1e6;
            total+=config_.appearance_weight*distance;weight+=config_.appearance_weight;
        }
        return total/weight+(class_mismatch?.15:0.);
    }
    void associate(std::vector<TrackObservation>& obs,const std::vector<int>& ids,std::vector<bool>& used,double stamp,bool second) {
        std::vector<int> rows;
        for(size_t i=0;i<tracks_.size();++i) if(!tracks_[i].updated && (!second || (tracks_[i].hits>=config_.min_hits && stamp-tracks_[i].last_seen<=.5))) rows.push_back(i);
        std::vector<std::vector<double>> c(rows.size(),std::vector<double>(ids.size()));
        for(size_t r=0;r<rows.size();++r)for(size_t j=0;j<ids.size();++j)c[r][j]=cost(tracks_[rows[r]],obs[ids[j]],second);
        auto matches=assign_tracks(c,config_.match);
        for(size_t r=0;r<matches.size();++r) if(matches[r]>=0) {
            int i=ids[matches[r]];correct(tracks_[rows[r]],obs[i],stamp);used[i]=true;
        }
    }
    void correct(Track& t,TrackObservation& o,double stamp) {
        Eigen::Matrix<double,7,1> z;z<<o.box.x+o.box.width/2,o.box.y+o.box.height/2,o.box.width,o.box.height,o.position;
        if(!t.hits) t.x.head<4>()=z.head<4>();
        if(o.has_position&&!t.has_position) {
            t.x.segment<3>(4)=o.position; t.x.segment<3>(11).setZero();
            // Reset stale metric cross-covariance on first/reacquired geometry.
            for(int axis=0;axis<3;++axis) for(int index:{4+axis,11+axis}) {
                t.covariance.row(index).setZero(); t.covariance.col(index).setZero();
                t.covariance(index,index)=index<7 ? config_.measurement_variance : 10.;
            }
        }
        int n=o.has_position?7:4;
        Eigen::MatrixXd h=Eigen::MatrixXd::Zero(n,14); h.leftCols(n).setIdentity();
        Eigen::MatrixXd r=Eigen::MatrixXd::Identity(n,n)*4.;
        if(o.has_position)r.bottomRightCorner(3,3)=Eigen::Matrix3d::Identity()*config_.measurement_variance;
        Eigen::MatrixXd s=h*t.covariance*h.transpose()+r;
        Eigen::MatrixXd k=t.covariance*h.transpose()*s.ldlt().solve(Eigen::MatrixXd::Identity(n,n));
        t.x+=k*(z.head(n)-h*t.x);
        Eigen::Matrix<double,14,14> a=Eigen::Matrix<double,14,14>::Identity()-k*h;
        t.covariance=a*t.covariance*a.transpose()+k*r*k.transpose(); // Joseph form
        if(o.has_position)t.last_position=stamp;
        t.has_position=t.has_position||o.has_position;t.hits++;t.last_seen=stamp;t.updated=true;t.score=o.score;
        if(!o.appearance.empty()) {
            if(t.appearance.size()!=o.appearance.size())t.appearance=o.appearance;
            else for(size_t i=0;i<t.appearance.size();++i)t.appearance[i]=.9f*t.appearance[i]+.1f*o.appearance[i];
            normalize_feature(t.appearance);
        }
        if(o.class_id==t.class_id) {
            t.pending_class=-1; t.pending_class_hits=0;
        } else if(o.score>=config_.high) {
            if(t.pending_class!=o.class_id) {t.pending_class=o.class_id;t.pending_class_hits=0;}
            if(++t.pending_class_hits>=config_.class_switch_hits) {
                t.class_id=o.class_id;t.pending_class=-1;t.pending_class_hits=0;
            }
        } else {
            t.pending_class=-1;t.pending_class_hits=0;
        }
        // Association attaches identity only. A track's historical label must
        // never overwrite the detector's class or inherit its confidence.
        if(t.hits>=config_.min_hits) o.track_id=t.id;
    }
    TrackerConfig config_; std::vector<Track> tracks_; int next_id_=1;
    double previous_=std::numeric_limits<double>::quiet_NaN();
};
} // namespace yoloe_infer
