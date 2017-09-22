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
  pnh_("~")
{

}

laser_filters::ScanValidator::~ScanValidator()
{

}

bool laser_filters::ScanValidator::configure()
{
  // Setup default values
  pnh_.param<float>("violation_percentage", violation_percentage_, 0.1);
  pnh_.param<float>("contour_tolerance", contour_tolerance_, 0.02);
  pnh_.param<std::string>("frame_id", frame_id_, "sick_lidar");
  pnh_.param<float>("angle_min", angle_min_);
  pnh_.param<float>("angle_max", angle_max_);
  pnh_.param<float>("angle_increment", angle_increment_);
  pnh_.param<float>("range_min", range_min_);
  pnh_.param<float>("range_max", range_max_);
  pnh_.param<std::vector<float> >("contour", contour_);

  return true;
}

bool laser_filters::ScanValidator::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  // Copy input_scan data
  output_scan = input_scan;

  if(!checkLaserConfig(input_scan))
  {
    return false;
  }

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

  // Stop laserscan from propagating to next filter chain
  if(cur_count >= number_threshold) {
    int errorPercentage = static_cast<int>(static_cast<float>(cur_count) / static_cast<float>(input_scan.ranges.size()) * 100.0);
    ROS_ERROR_THROTTLE(5.0, "%d percent of the scan readings are smaller than expected, lidar might be occluded", errorPercentage);
    return false;
  }

  return true;
}

bool laser_filters::ScanValidator::checkLaserConfig(
    const sensor_msgs::LaserScan& scan)
{
  if(scan.ranges.size() != contour_.size() ||
     !srs::BasicMath::equal<float>(scan.angle_min, angle_min_) ||
     !srs::BasicMath::equal<float>(scan.angle_max, angle_max_) ||
     !srs::BasicMath::equal<float>(scan.angle_increment, angle_increment_) ||
     !srs::BasicMath::equal<float>(scan.range_min, range_min_) ||
     !srs::BasicMath::equal<float>(scan.range_max, range_max_))
  {
    ROS_ERROR_THROTTLE(5.0, "Laserscan received violates the validator laserscan params");
    return false;
  }

  return true;
}

