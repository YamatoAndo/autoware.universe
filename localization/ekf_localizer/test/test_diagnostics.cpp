// Copyright 2023 Autoware Foundation
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

#include "ekf_localizer/diagnostics.hpp"

#include <gtest/gtest.h>

TEST(TestEkfDiagnostics, CheckProcessActivated)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;

  bool is_activated = true;
  checkProcessActivated(stat, &is_activated);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_activated = false;
  checkProcessActivated(stat, &is_activated);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, checkMeasurementUpdated)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;

  const std::string measurement_type = "pose";  // not effect for stat.level
  const size_t no_update_count_threshold_warn = 50;
  const size_t no_update_count_threshold_error = 250;

  size_t no_update_count = 0;
  checkMeasurementUpdated(
    stat, measurement_type, &no_update_count, &no_update_count_threshold_warn,
    &no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 1;
  checkMeasurementUpdated(
    stat, measurement_type, &no_update_count, &no_update_count_threshold_warn,
    &no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 49;
  checkMeasurementUpdated(
    stat, measurement_type, &no_update_count, &no_update_count_threshold_warn,
    &no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  no_update_count = 50;
  checkMeasurementUpdated(
    stat, measurement_type, &no_update_count, &no_update_count_threshold_warn,
    &no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  no_update_count = 249;
  checkMeasurementUpdated(
    stat, measurement_type, &no_update_count, &no_update_count_threshold_warn,
    &no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  no_update_count = 250;
  checkMeasurementUpdated(
    stat, measurement_type, &no_update_count, &no_update_count_threshold_warn,
    &no_update_count_threshold_error);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

TEST(TestEkfDiagnostics, CheckMeasurementQueueSize)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;

  const std::string measurement_type = "pose";  // not effect for stat.level

  size_t queue_size_ptr = 0;
  checkMeasurementQueueSize(stat, measurement_type, &queue_size_ptr);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  queue_size_ptr = 1;
  checkMeasurementQueueSize(stat, measurement_type, &queue_size_ptr);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

TEST(TestEkfDiagnostics, CheckMeasurementDelayGate)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;

  const std::string measurement_type = "pose";  // not effect for stat.level
  const double delay_time = 0.1;                // not effect for stat.level
  const double delay_time_threshold = 1.0;      // not effect for stat.level

  bool is_passed_delay_gate = true;
  checkMeasurementDelayGate(
    stat, measurement_type, &is_passed_delay_gate, &delay_time, &delay_time_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_passed_delay_gate = false;
  checkMeasurementDelayGate(
    stat, measurement_type, &is_passed_delay_gate, &delay_time, &delay_time_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}

TEST(TestEkfDiagnostics, CheckMeasurementMahalanobisGate)
{
  diagnostic_updater::DiagnosticStatusWrapper stat;

  const std::string measurement_type = "pose";        // not effect for stat.level
  const double mahalabobis_distance = 0.1;            // not effect for stat.level
  const double mahalabobis_distance_threshold = 1.0;  // not effect for stat.level

  bool is_passed_mahalabobis_gate = true;
  checkMeasurementMahalanobisGate(
    stat, measurement_type, &is_passed_mahalabobis_gate, &mahalabobis_distance,
    &mahalabobis_distance_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  is_passed_mahalabobis_gate = false;
  checkMeasurementMahalanobisGate(
    stat, measurement_type, &is_passed_mahalabobis_gate, &mahalabobis_distance,
    &mahalabobis_distance_threshold);
  EXPECT_EQ(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
}
