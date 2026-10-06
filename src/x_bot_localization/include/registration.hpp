#pragma once
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>

namespace xbot {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
struct Result {
  bool valid = false;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  double rmse = std::numeric_limits<double>::infinity();
  double overlap = 0.;
};
inline Cloud::Ptr voxel(const Cloud::ConstPtr &cloud, float size) {
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(size, size, size);
  filter.filter(*out);
  return out;
}
inline Result align(const Cloud::ConstPtr &scan, const Cloud::ConstPtr &target,
                    const Eigen::Matrix4f &guess, double max_rmse=.15, double min_overlap=.65) {
  Result result;
  result.pose = guess;
  if (scan->size() < 100 || target->size() < 100 || !guess.allFinite()) return result;
  for (float leaf : {.4f, .2f, .1f}) {
    auto source = voxel(scan, leaf), map = voxel(target, leaf);
    if (source->size() < 30 || map->size() < 30) return result;
    pcl::IterativeClosestPoint<Point, Point> icp;
    icp.setInputSource(source);
    icp.setInputTarget(map);
    icp.setMaximumIterations(40);
    icp.setMaxCorrespondenceDistance(leaf*3);
    icp.setTransformationEpsilon(1e-7);
    icp.setEuclideanFitnessEpsilon(1e-6);
    Cloud registered;
    icp.align(registered, result.pose);
    if (!icp.hasConverged()) return result;
    result.pose = icp.getFinalTransformation();
  }
  auto source = voxel(scan, .1f), map = voxel(target, .1f);
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, result.pose);
  pcl::KdTreeFLANN<Point> tree;
  tree.setInputCloud(map);
  std::vector<int> ids(1);
  std::vector<float> distances(1);
  double sum = 0;
  size_t inliers = 0;
  for (const auto &p : transformed) {
    if (tree.nearestKSearch(p, 1, ids, distances) && distances[0] < .09f) {
      ++inliers;
      sum += distances[0];
    }
  }
  result.overlap = double(inliers) / transformed.size();
  result.rmse = inliers ? std::sqrt(sum / inliers) : std::numeric_limits<double>::infinity();
  const Eigen::Matrix4f delta = result.pose * guess.inverse();
  const double angle = Eigen::AngleAxisf(delta.block<3,3>(0,0)).angle();
  result.valid = result.pose.allFinite() && result.rmse <= max_rmse && result.overlap >= min_overlap
      && delta.block<3,1>(0,3).norm() < 2.0 && std::abs(angle) < .6;
  return result;
}
}
