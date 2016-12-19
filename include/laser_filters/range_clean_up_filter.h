/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, JSK (The University of Tokyo).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_SCAN_RANGE_CLEAN_UP_FILTER_H
#define LASER_SCAN_RANGE_CLEAN_UP_FILTER_H
/**
\author Yohei Kakiuchi
@b ScanRangeFilter takes input scans and filters within indicated range.
**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"
#include <cmath>

namespace laser_filters
{

class LaserScanRangeCleanUpFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  bool configure()
  {
    return true;
  }

  virtual ~LaserScanRangeCleanUpFilter()
  {

  }

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    // ROS_WARN("Cleaning up ranges.");
    double EPSILON = 0.001;
    filtered_scan = input_scan;
    for (unsigned int i=0;
         i < input_scan.ranges.size();
         i++) // Need to check ever reading in the current scan
    {
      {
      if (!std::isfinite(filtered_scan.ranges[i])
        || filtered_scan.ranges[i] < filtered_scan.range_min
        || filtered_scan.ranges[i] > filtered_scan.range_max)
        {
          filtered_scan.ranges[i] = filtered_scan.range_max - EPSILON;
        }
      }
    }

    return true;
  }
} ;

}

#endif // LASER_SCAN_RANGE_CLEAN_UP_FILTER_H
