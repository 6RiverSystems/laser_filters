/*
 * (c) Copyright 2015-2016 6 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 *
 * scan_validator.h
 *
 * created: 09/12/2017
 */

#include <math.h>
#include <ros/ros.h>
#include <srslib_framework/math/BasicMath.hpp>
#include "laser_filters/scan_validator.h"

laser_filters::ScanValidator::ScanValidator() :
  has_checked_laser_config_(false)
{

}

laser_filters::ScanValidator::~ScanValidator()
{

}

bool laser_filters::ScanValidator::configure()
{
  // Setup default values
  getParam("violation_percentage", violation_percentage_);
  getParam("contour_tolerance", contour_tolerance_);
  getParam("frame_id", frame_id_);
  getParam("angle_min", angle_min_);
  getParam("angle_max", angle_max_);
  getParam("angle_increment", angle_increment_);
  getParam("range_min", range_min_);
  getParam("range_max", range_max_);
  getParam("contour", contour_);

  return true;
}

bool laser_filters::ScanValidator::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  // One-time check for laserscan configuration
  if(!has_checked_laser_config_)
  {
    if(!checkLaserConfig(input_scan))
    {
      return false;
    } else {
      has_checked_laser_config_ = true;
    }
  }

  // Copy input_scan data
  output_scan = input_scan;

  int number_threshold = static_cast<int>(violation_percentage_ * input_scan.ranges.size());
  int cur_count = 0;

  // Traverse each point
  for(auto i = 0; i < input_scan.ranges.size(); i++)
  {
    // Counts for the reading is NaN or smaller than contour range
    // Make the contour slightly smaller to avoid false positive case
    if(std::isnan(input_scan.ranges[i]) ||
       input_scan.ranges[i] < (contour_[i] - contour_tolerance_))
    {
      cur_count += 1;
    }
  }

  int occlusionPercentage = static_cast<int>((static_cast<float>(cur_count) * 100) / static_cast<float>(input_scan.ranges.size()));

  // Stop laserscan from propagating to next filter chain
  if(cur_count >= number_threshold) {
    ROS_ERROR_THROTTLE(5.0, "%d percent of the scan readings are smaller than expected, lidar might be occluded", occlusionPercentage);
    return false;
  }

  ROS_DEBUG_THROTTLE(60.0, "%d percent laserscan occlusion", occlusionPercentage);
  return true;
}

bool laser_filters::ScanValidator::checkLaserConfig(
    const sensor_msgs::LaserScan& scan)
{
  const double EPS = 0.0001;

  if(scan.ranges.size() != contour_.size() ||
     !srs::BasicMath::equal<double>(scan.angle_min, angle_min_, EPS) ||
     !srs::BasicMath::equal<double>(scan.angle_max, angle_max_, EPS) ||
     !srs::BasicMath::equal<double>(scan.angle_increment, angle_increment_, EPS) ||
     !srs::BasicMath::equal<double>(scan.range_min, range_min_, EPS) ||
     !srs::BasicMath::equal<double>(scan.range_max, range_max_, EPS))
  {
    ROS_ERROR_ONCE("Input laserscan configuration is different from default setting");
    return false;
  }

  return true;
}

