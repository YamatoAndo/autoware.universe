// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef MULTI_OBJECT_TRACKER__TRACKER__MODEL__BICYCLE_TRACKER_HPP_
#define MULTI_OBJECT_TRACKER__TRACKER__MODEL__BICYCLE_TRACKER_HPP_

#include "multi_object_tracker/tracker/model/tracker_base.hpp"

#include <kalman_filter/kalman_filter.hpp>

class BicycleTracker : public Tracker
{
private:
  autoware_auto_perception_msgs::msg::DetectedObject object_;
  rclcpp::Logger logger_;

private:
  KalmanFilter ekf_;
  rclcpp::Time last_update_time_;
  enum IDX { X = 0, Y = 1, YAW = 2, VEL = 3, SLIP = 4 };
  struct EkfParams
  {
    char dim_x = 5;
    float q_stddev_acc_long;
    float q_stddev_acc_lat;
    float q_stddev_yaw_rate_min;
    float q_stddev_yaw_rate_max;
    float q_cov_slip_rate_min;
    float q_cov_slip_rate_max;
    float q_max_slip_angle;
    float p0_cov_vel;
    float p0_cov_slip;
    float r_cov_x;
    float r_cov_y;
    float r_cov_yaw;
    float p0_cov_x;
    float p0_cov_y;
    float p0_cov_yaw;
  } ekf_params_;

  double max_vel_;
  double max_slip_;
  double lf_;
  double lr_;
  float z_;

private:
  struct BoundingBox
  {
    double length;
    double width;
    double height;
  };
  BoundingBox bounding_box_;
  BoundingBox last_input_bounding_box_;

public:
  BicycleTracker(
    const rclcpp::Time & time, const autoware_auto_perception_msgs::msg::DetectedObject & object,
    const geometry_msgs::msg::Transform & self_transform);

  bool predict(const rclcpp::Time & time) override;
  bool predict(const double dt, KalmanFilter & ekf) const;
  bool measure(
    const autoware_auto_perception_msgs::msg::DetectedObject & object, const rclcpp::Time & time,
    const geometry_msgs::msg::Transform & self_transform) override;
  bool measureWithPose(const autoware_auto_perception_msgs::msg::DetectedObject & object);
  bool measureWithShape(const autoware_auto_perception_msgs::msg::DetectedObject & object);
  bool getTrackedObject(
    const rclcpp::Time & time,
    autoware_auto_perception_msgs::msg::TrackedObject & object) const override;
  virtual ~BicycleTracker() {}
};

#endif  // MULTI_OBJECT_TRACKER__TRACKER__MODEL__BICYCLE_TRACKER_HPP_
