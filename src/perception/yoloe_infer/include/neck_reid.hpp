#pragma once
#include "bot_sort.hpp"
namespace yoloe_infer {
// Average pool an original-image ROI on a planar NCHW neck tensor, undoing
// the exact resize/letterbox mapping used by TensorRT preprocessing.
inline std::vector<float> pool_neck(const float* data,int channels,int height,int width,
    const cv::Rect2f& box,float scale,int pad_x,int pad_y,int input_w,int input_h) {
    if(!data||channels<=0||height<=0||width<=0||input_w<=0||input_h<=0||scale<=0) return {};
    int x0=std::clamp(int(std::floor((box.x*scale+pad_x)*width/input_w)),0,width);
    int y0=std::clamp(int(std::floor((box.y*scale+pad_y)*height/input_h)),0,height);
    int x1=std::clamp(int(std::ceil(((box.x+box.width)*scale+pad_x)*width/input_w)),0,width);
    int y1=std::clamp(int(std::ceil(((box.y+box.height)*scale+pad_y)*height/input_h)),0,height);
    if(x1<=x0||y1<=y0)return {};
    std::vector<float> feature(channels,0);
    for(int c=0;c<channels;++c)for(int y=y0;y<y1;++y)for(int x=x0;x<x1;++x)
        feature[c]+=data[(c*height+y)*width+x]/((x1-x0)*(y1-y0));
    normalize_feature(feature);return feature;
}
}
