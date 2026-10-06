#include <gtest/gtest.h>
#include "registration.hpp"
#include <random>

TEST(Registration, SeededTransformAndBadOverlap) {
  auto map=std::make_shared<xbot::Cloud>();
  std::mt19937 random(123);
  std::uniform_real_distribution<float> uniform(-3,3);
  for (int i=0;i<4000;++i) {
    xbot::Point p;
    p.x=uniform(random); p.y=uniform(random); p.z=uniform(random)*.5;
    map->push_back(p);
  }
  Eigen::Matrix4f truth=Eigen::Matrix4f::Identity();
  truth.block<3,3>(0,0)=Eigen::AngleAxisf(.15f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  truth(0,3)=.3f; truth(1,3)=-.2f;
  auto scan=std::make_shared<xbot::Cloud>();
  pcl::transformPointCloud(*map,*scan,truth.inverse().eval());
  auto guess=truth; guess(0,3)+=.08f;
  auto result=xbot::align(scan,map,guess);
  ASSERT_TRUE(result.valid);
  EXPECT_LT((result.pose-truth).norm(),.03);
  EXPECT_GT(result.overlap,.95);
  guess(0,3)=100;
  EXPECT_FALSE(xbot::align(scan,map,guess).valid);
  auto empty=std::make_shared<xbot::Cloud>();
  EXPECT_FALSE(xbot::align(empty,map,truth).valid);
}
