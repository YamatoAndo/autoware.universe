// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher/util_func.hpp"

#include "ndt_scan_matcher/matrix_type.hpp"

static std::random_device seed_gen;

double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2)
{
  return std::sqrt(
    std::pow(p1.x - p2.x, 2.0) + std::pow(p1.y - p2.y, 2.0) + std::pow(p1.z - p2.z, 2.0));
}

double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad > M_PI) {
    diff_rad = diff_rad - 2 * M_PI;
  } else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

Eigen::Map<const RowMatrixXd> makeEigenCovariance(const std::array<double, 36> & covariance)
{
  return Eigen::Map<const RowMatrixXd>(covariance.data(), 6, 6);
}

// x: roll, y: pitch, z: yaw
geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::Pose & pose)
{
  geometry_msgs::msg::Vector3 rpy;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseStamped & pose)
{
  return getRPY(pose.pose);
}

geometry_msgs::msg::Vector3 getRPY(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
{
  return getRPY(pose.pose.pose);
}


geometry_msgs::msg::Twist calcTwist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b)
{
  const rclcpp::Duration dt = rclcpp::Time(pose_b.header.stamp) - rclcpp::Time(pose_a.header.stamp);
  const double dt_s = dt.seconds();

  if (dt_s == 0) {
    return geometry_msgs::msg::Twist();
  }

  const auto pose_a_rpy = getRPY(pose_a);
  const auto pose_b_rpy = getRPY(pose_b);

  geometry_msgs::msg::Vector3 diff_xyz;
  geometry_msgs::msg::Vector3 diff_rpy;

  diff_xyz.x = pose_b.pose.position.x - pose_a.pose.position.x;
  diff_xyz.y = pose_b.pose.position.y - pose_a.pose.position.y;
  diff_xyz.z = pose_b.pose.position.z - pose_a.pose.position.z;
  diff_rpy.x = calcDiffForRadian(pose_b_rpy.x, pose_a_rpy.x);
  diff_rpy.y = calcDiffForRadian(pose_b_rpy.y, pose_a_rpy.y);
  diff_rpy.z = calcDiffForRadian(pose_b_rpy.z, pose_a_rpy.z);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = diff_xyz.x / dt_s;
  twist.linear.y = diff_xyz.y / dt_s;
  twist.linear.z = diff_xyz.z / dt_s;
  twist.angular.x = diff_rpy.x / dt_s;
  twist.angular.y = diff_rpy.y / dt_s;
  twist.angular.z = diff_rpy.z / dt_s;

  return twist;
}

void getNearestTimeStampPose(
  const std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> &
    pose_cov_msg_ptr_array,
  const rclcpp::Time & time_stamp,
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & output_old_pose_cov_msg_ptr,
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr & output_new_pose_cov_msg_ptr)
{
  for (const auto & pose_cov_msg_ptr : pose_cov_msg_ptr_array) {
    output_new_pose_cov_msg_ptr =
      std::const_pointer_cast<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_msg_ptr);
    const rclcpp::Time pose_time_stamp = output_new_pose_cov_msg_ptr->header.stamp;
    if (pose_time_stamp > time_stamp) {
      // TODO(Tier IV): refactor
      if (pose_time_stamp.seconds() == 0.0) {
        output_old_pose_cov_msg_ptr = output_new_pose_cov_msg_ptr;
      }
      break;
    }
    output_old_pose_cov_msg_ptr = output_new_pose_cov_msg_ptr;
  }
}

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp)
{
  const rclcpp::Time pose_a_time_stamp = pose_a.header.stamp;
  const rclcpp::Time pose_b_time_stamp = pose_b.header.stamp;
  if (
    (pose_a_time_stamp.seconds() == 0.0) || (pose_b_time_stamp.seconds() == 0.0) ||
    (time_stamp.seconds() == 0.0)) {
    return geometry_msgs::msg::PoseStamped();
  }

  const auto twist = calcTwist(pose_a, pose_b);
  const double dt = (time_stamp - pose_a_time_stamp).seconds();

  const auto pose_a_rpy = getRPY(pose_a);

  geometry_msgs::msg::Vector3 xyz;
  geometry_msgs::msg::Vector3 rpy;
  xyz.x = pose_a.pose.position.x + twist.linear.x * dt;
  xyz.y = pose_a.pose.position.y + twist.linear.y * dt;
  xyz.z = pose_a.pose.position.z + twist.linear.z * dt;
  rpy.x = pose_a_rpy.x + twist.angular.x * dt;
  rpy.y = pose_a_rpy.y + twist.angular.y * dt;
  rpy.z = pose_a_rpy.z + twist.angular.z * dt;

  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

  geometry_msgs::msg::PoseStamped pose;
  pose.header = pose_a.header;
  pose.header.stamp = time_stamp;
  pose.pose.position.x = xyz.x;
  pose.pose.position.y = xyz.y;
  pose.pose.position.z = xyz.z;
  pose.pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

geometry_msgs::msg::PoseStamped interpolatePose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const rclcpp::Time & time_stamp)
{
  geometry_msgs::msg::PoseStamped tmp_pose_a;
  tmp_pose_a.header = pose_a.header;
  tmp_pose_a.pose = pose_a.pose.pose;

  geometry_msgs::msg::PoseStamped tmp_pose_b;
  tmp_pose_b.header = pose_b.header;
  tmp_pose_b.pose = pose_b.pose.pose;

  return interpolatePose(tmp_pose_a, tmp_pose_b, time_stamp);
}

void popOldPose(
  std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr> &
    pose_cov_msg_ptr_array,
  const rclcpp::Time & time_stamp)
{
  while (!pose_cov_msg_ptr_array.empty()) {
    if (rclcpp::Time(pose_cov_msg_ptr_array.front()->header.stamp) >= time_stamp) {
      break;
    }
    pose_cov_msg_ptr_array.pop_front();
  }
}

Eigen::Affine3d fromRosPoseToEigen(const geometry_msgs::msg::Pose & ros_pose)
{
  Eigen::Affine3d eigen_pose;
  tf2::fromMsg(ros_pose, eigen_pose);
  return eigen_pose;
}

std::vector<geometry_msgs::msg::Pose> createRandomPoseArray(
  const geometry_msgs::msg::PoseWithCovarianceStamped & base_pose_with_cov,
  const size_t particle_num)
{
  std::default_random_engine engine(seed_gen());
  const Eigen::Map<const RowMatrixXd> covariance =
    makeEigenCovariance(base_pose_with_cov.pose.covariance);

  std::normal_distribution<> x_distribution(0.0, std::sqrt(covariance(0, 0)));
  std::normal_distribution<> y_distribution(0.0, std::sqrt(covariance(1, 1)));
  std::normal_distribution<> z_distribution(0.0, std::sqrt(covariance(2, 2)));
  std::normal_distribution<> roll_distribution(0.0, std::sqrt(covariance(3, 3)));
  std::normal_distribution<> pitch_distribution(0.0, std::sqrt(covariance(4, 4)));
  std::normal_distribution<> yaw_distribution(0.0, std::sqrt(covariance(5, 5)));

  const auto base_rpy = getRPY(base_pose_with_cov);

  std::vector<geometry_msgs::msg::Pose> poses;
  for (size_t i = 0; i < particle_num; ++i) {
    geometry_msgs::msg::Vector3 xyz;
    geometry_msgs::msg::Vector3 rpy;

    xyz.x = base_pose_with_cov.pose.pose.position.x + x_distribution(engine);
    xyz.y = base_pose_with_cov.pose.pose.position.y + y_distribution(engine);
    xyz.z = base_pose_with_cov.pose.pose.position.z + z_distribution(engine);
    rpy.x = base_rpy.x + roll_distribution(engine);
    rpy.y = base_rpy.y + pitch_distribution(engine);
    rpy.z = base_rpy.z + yaw_distribution(engine);

    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

    geometry_msgs::msg::Pose pose;
    pose.position.x = xyz.x;
    pose.position.y = xyz.y;
    pose.position.z = xyz.z;
    pose.orientation = tf2::toMsg(tf_quaternion);

    poses.push_back(pose);
  }

  return poses;
}

bool isLocalOptimalSolutionOscillation(
  const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &
    result_pose_matrix_array,
  const float oscillation_threshold, const float inversion_vector_threshold)
{
  bool prev_oscillation = false;
  int oscillation_cnt = 0;
  for (size_t i = 2; i < result_pose_matrix_array.size(); ++i) {
    const Eigen::Vector3f current_pose = result_pose_matrix_array.at(i).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_pose = result_pose_matrix_array.at(i - 1).block(0, 3, 3, 1);
    const Eigen::Vector3f prev_prev_pose = result_pose_matrix_array.at(i - 2).block(0, 3, 3, 1);
    const auto current_vec = (current_pose - prev_pose).normalized();
    const auto prev_vec = (prev_pose - prev_prev_pose).normalized();
    const bool oscillation = prev_vec.dot(current_vec) < inversion_vector_threshold;
    if (prev_oscillation && oscillation) {
      if (oscillation_cnt > oscillation_threshold) {
        return true;
      }
      ++oscillation_cnt;
    } else {
      oscillation_cnt = 0;
    }
    prev_oscillation = oscillation;
  }
  return false;
}

geometry_msgs::msg::TransformStamped identityTransformStamped(
  const builtin_interfaces::msg::Time & timestamp, const std::string & header_frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = timestamp;
  transform.header.frame_id = header_frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform.rotation = tier4_autoware_utils::createQuaternion(0.0, 0.0, 0.0, 1.0);
  transform.transform.translation = tier4_autoware_utils::createTranslation(0.0, 0.0, 0.0);
  return transform;
}

tier4_debug_msgs::msg::Float32Stamped makeFloat32Stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = tier4_debug_msgs::msg::Float32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}

tier4_debug_msgs::msg::Int32Stamped makeInt32Stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = tier4_debug_msgs::msg::Int32Stamped;
  return tier4_debug_msgs::build<T>().stamp(stamp).data(data);
}
