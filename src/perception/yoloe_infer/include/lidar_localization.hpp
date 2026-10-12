#pragma once
#include "bot_sort.hpp"
namespace yoloe_infer {
struct ProjectedLidarPoint {int u,v; double depth; Eigen::Vector3d world;};
inline bool lidar_position(const cv::Rect2f& box,const cv::Mat& mask,
    const std::vector<ProjectedLidarPoint>& points,int width,int height,
    Eigen::Vector3d& result,int minimum=5,double cluster_gap=.3) {
    if(width<=0||height<=0||minimum<1) return false;
    std::vector<double> front(width*height,std::numeric_limits<double>::infinity());
    for(const auto& p:points) if(p.u>=0&&p.u<width&&p.v>=0&&p.v<height&&p.depth>0&&std::isfinite(p.depth)&&p.world.allFinite())
        front[p.v*width+p.u]=std::min(front[p.v*width+p.u],p.depth);
    std::vector<const ProjectedLidarPoint*> selected;
    int left=std::max(0,int(box.x)),top=std::max(0,int(box.y));
    for(const auto& p:points) {
        if(p.u<0||p.u>=width||p.v<0||p.v>=height||!p.world.allFinite()||!std::isfinite(p.depth)||p.depth<=0||
           !box.contains(cv::Point2f(p.u,p.v))||p.depth>front[p.v*width+p.u]+.15)continue;
        if(!mask.empty()) {
            int x=p.u-left,y=p.v-top;
            if(mask.type()!=CV_8UC1||x<0||y<0||x>=mask.cols||y>=mask.rows||!mask.at<uint8_t>(y,x))continue;
        }
        selected.push_back(&p);
    }
    std::sort(selected.begin(),selected.end(),[](auto a,auto b){return a->depth<b->depth;});
    // Nearest sufficiently populated depth component rejects isolated foreground
    // returns and background through holes in the instance mask.
    for(size_t begin=0;begin<selected.size();) {
        size_t end=begin+1;
        while(end<selected.size()&&selected[end]->depth-selected[end-1]->depth<=cluster_gap)++end;
        if(int(end-begin)>=minimum) {
            for(int axis=0;axis<3;++axis) {
                std::vector<double> values;
                for(size_t j=begin;j<end;++j) values.push_back(selected[j]->world[axis]);
                auto mid=values.begin()+values.size()/2;
                std::nth_element(values.begin(),mid,values.end());result[axis]=*mid;
            }
            return true;
        }
        begin=end;
    }
    return false;
}
}
