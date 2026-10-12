#include <gtest/gtest.h>
#include "registration.hpp"
#include <random>

TEST(Registration, CorrectionFilterReducesStationaryJitter) {
  auto filtered=Eigen::Matrix4f::Identity().eval();
  double raw_motion=0, filtered_motion=0;
  float previous_raw=0;
  for (int i=0;i<100;++i) {
    auto measurement=Eigen::Matrix4f::Identity().eval();
    measurement(0,3)=(i%2 ? .01f : -.01f);
    measurement.block<3,3>(0,0)=Eigen::AngleAxisf(i%2 ? .006f : -.006f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
    auto next=xbot::smooth_correction(filtered,measurement,.05,.5);
    raw_motion+=std::abs(measurement(0,3)-previous_raw);
    filtered_motion+=std::abs(next(0,3)-filtered(0,3));
    EXPECT_NEAR((next.block<3,3>(0,0).determinant()),1.,1e-5);
    previous_raw=measurement(0,3); filtered=next;
  }
  EXPECT_LT(filtered_motion,raw_motion*.1);
}

TEST(Registration, CorrectionFilterConvergesWithoutOvershootAndCanBeDisabled) {
  auto previous=Eigen::Matrix4f::Identity().eval(), measurement=previous;
  measurement(0,3)=1;
  measurement.block<3,3>(0,0)=Eigen::AngleAxisf(.4f,Eigen::Vector3f::UnitZ()).toRotationMatrix();
  EXPECT_EQ(xbot::smooth_correction(previous,measurement,.05,0),measurement);
  EXPECT_EQ(xbot::smooth_correction(previous,measurement,0,.5),previous);
  for (int i=0;i<100;++i) {
    auto next=xbot::smooth_correction(previous,measurement,.05,.5);
    EXPECT_GE(next(0,3),previous(0,3)); EXPECT_LE(next(0,3),1);
    previous=next;
  }
  EXPECT_LT((previous-measurement).norm(),.001);
}

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
