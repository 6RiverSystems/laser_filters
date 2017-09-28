/*
 * (c) Copyright 2015-2016 6 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 *
 * radius_search_filter.h
 *
 * created: 11/02/2016
 */

#include <set>
#include <math.h>
#include <ros/ros.h>
#include "laser_filters/radius_search_filter.h"

laser_filters::RadiusSearchFilter::RadiusSearchFilter(){
}

laser_filters::RadiusSearchFilter::~RadiusSearchFilter(){
}

bool laser_filters::RadiusSearchFilter::configure()
{
  // Setup default values
  neighbor_num_ = 3;
  threshold_num_ = 3;
  threshold_ratio_ = 0.10;

  // launch values from parameter server
  getParam("neighbor_number", neighbor_num_);
  getParam("threshold_number", threshold_num_);
  getParam("threshold_ratio",  threshold_ratio_);

  return true;
}

bool laser_filters::RadiusSearchFilter::update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan &output_scan)
{
  // copy input_scan data
  output_scan = input_scan;

  // declare a set for indexing point
  std::set<int> indices_to_delete;

  // traverse each point
  for(int i= 0; i< input_scan.ranges.size(); i++)
  {
    // skip if the range of the point is NaN
    if(std::isnan(input_scan.ranges[i]))
    {
      continue;
    }

    int counter = 0;

    // adaptive threshold based on input_scan range value
    // the further the point, the larger the radius search is set
    double cur_threshold = ((int)(input_scan.ranges[i]) + 1) * threshold_ratio_;

    // for each point, visit its neighbors
    for(int j= -neighbor_num_; j< neighbor_num_ + 1; j++)
    {
      int neighbor_index = i + j;

      // skip out-of-bound points and the point itself
      // also skip if the range of the neighboring point is NaN
      if( neighbor_index< 0 || 
          neighbor_index >= (int)input_scan.ranges.size() ||
          neighbor_index == i ||
          std::isnan(input_scan.ranges[neighbor_index]))
      {
        continue;
      }

      // calculate the square distance between the point and one of the neighbor point
      // c^2 = a^2 + b^2 - 2 * a * b * cos(alpha)
      double cur_dist_sq = (input_scan.ranges[i] * input_scan.ranges[i]) + (input_scan.ranges[neighbor_index] * input_scan.ranges[neighbor_index]) - 2 * input_scan.ranges[i] * input_scan.ranges[neighbor_index] * cos((i - neighbor_index) * input_scan.angle_increment);

      if( cur_dist_sq <= cur_threshold * cur_threshold )
      {
        counter++;
      }

      if(counter >= threshold_num_)
      {
        break;
      }
    }

    // if a point is unqualified, add it to the set
    if(counter < threshold_num_)
    {
      indices_to_delete.insert(i);
    }
  }

  // assign range to NaN for each filtered point
  for( std::set<int>::iterator it = indices_to_delete.begin(); it != indices_to_delete.end(); it++)
  {
    output_scan.ranges[*it] = std::numeric_limits<float>::quiet_NaN();
  }

  return true;
}

