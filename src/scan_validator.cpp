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
#include "laser_filters/scan_validator.h"

laser_filters::ScanValidator::ScanValidator(){
}

laser_filters::ScanValidator::~ScanValidator(){
}

bool laser_filters::ScanValidator::configure()
{
  // Setup default values
  violation_percentage_ = 30.0;
  consecutive_scans_ = 5;
  contour_tolerance_ = 0.02;

  cur_scans_ = 0;

  getParam("violation_percentage", violation_percentage_);
  getParam("consecutive_scans", consecutive_scans_);
  getParam("contour", contour_);
  getParam("contour_tolerance", contour_tolerance_);

  return true;
}

bool laser_filters::ScanValidator::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  // Copy input_scan data
  output_scan = input_scan;

  if(input_scan.ranges.size() != contour_.size())
  {
    ROS_WARN_THROTTLE(10.0, "Laserscan size doesn't match the validator contour, skip validation step");
    return true;
  }

  int number_threshold = (int)(violation_percentage_ / 100.0 * input_scan.ranges.size());
  int cur_count = 0;

  // Traverse each point
  for(int i = 0; i < input_scan.ranges.size(); i++)
  {
    // If the reading is NaN or smaller than contour range
    // Make the contour slightly smaller to avoid false positive case
    if(isnan(input_scan.ranges[i]) || input_scan.ranges[i] < (contour_[i] - contour_tolerance_))
    {
      cur_count += 1;
    }
  }

  // If laser occlusion is caught, increase cur_scan_ or reset to zero otherwise
  if(cur_count >= number_threshold) {
    cur_scans_ += 1;
    // Stop laserscan from propagating to next filter chain
    if(cur_scans_ >= consecutive_scans_) {
      return false;
    }
  } else {
    cur_scans_ = 0;
  }

  return true;
}

