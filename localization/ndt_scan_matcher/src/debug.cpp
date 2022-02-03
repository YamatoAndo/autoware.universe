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

#include "ndt_scan_matcher/debug.hpp"

#include "ndt_scan_matcher/util_func.hpp"

// ref by http://takacity.blog.fc2.com/blog-entry-69.html
std_msgs::msg::ColorRGBA ExchangeColorCrc(double x)
{
  std_msgs::msg::ColorRGBA color;

  x = std::max(x, 0.0);
  x = std::min(x, 0.9999);

  if (x <= 0.25) {
    color.b = 1.0;
    color.g = std::sin(x * 2.0 * M_PI);
    color.r = 0;
  } else if (x > 0.25 && x <= 0.5) {
    color.b = std::sin(x * 2 * M_PI);
    color.g = 1.0;
    color.r = 0;
  } else if (x > 0.5 && x <= 0.75) {
    color.b = 0;
    color.g = 1.0;
    color.r = -std::sin(x * 2.0 * M_PI);
  } else {
    color.b = 0;
    color.g = -std::sin(x * 2.0 * M_PI);
    color.r = 1.0;
  }
  color.a = 0.999;
  return color;
}


visualization_msgs::msg::MarkerArray makeDebugMarkers(
  const builtin_interfaces::msg::Time & stamp, const std::string & map_frame_,
  const geometry_msgs::msg::Vector3 & scale, const Particle & particle, const size_t i)
{
  // TODO(Tier IV): getNumSubscribers
  // TODO(Tier IV): clear old object
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = scale;
  marker.id = i;

  marker.ns = "initial_pose_transform_probability_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_iteration_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "initial_pose_index_color_marker";
  marker.pose = particle.initial_pose;
  marker.color = ExchangeColorCrc((1.0 * i) / 100);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_transform_probability_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc(particle.score / 4.5);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_iteration_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc((1.0 * particle.iteration) / 30.0);
  marker_array.markers.push_back(marker);

  marker.ns = "result_pose_index_color_marker";
  marker.pose = particle.result_pose;
  marker.color = ExchangeColorCrc((1.0 * i) / 100);
  marker_array.markers.push_back(marker);

  return marker_array;
}
