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

  cur_scans_ = 0;

  getParam("violation_percentage", violation_percentage_);
  getParam("consecutive_scans", consecutive_scans_);

  return true;
}

bool laser_filters::ScanValidator::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  // copy input_scan data
  output_scan = input_scan;

  int number_threshold = (int)(violation_percentage_ / 100.0 * input_scan.ranges.size());
  int cur_count = 0;

  // traverse each point
  for(int i= 0; i< input_scan.ranges.size(); i++)
  {
    // if NaN 
    if(isnan(input_scan.ranges[i]) || input_scan.ranges[i] < 0.50)
    {
      cur_count += 1;
    }
  }

  if(cur_count >= number_threshold) {
    cur_scans_ += 1;
    if(cur_scans_ >= consecutive_scans_) {
      // reset counter
      cur_scans_ -= 1;
      return false;
    }
  } else {
    cur_scans_ = 0;
  }

  return true;
}

