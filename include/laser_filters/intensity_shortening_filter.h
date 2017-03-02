/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef LASER_SCAN_INTENSITY_SHORTENING_FILTER_H
#define LASER_SCAN_INTENSITY_SHORTENING_FILTER_H
/**
\author Daniel Grieneisen
@b ScanIntensityFilter takes input scans and fiters out that are not within the specified range. The filtered out readings are set at >max_range in order to invalidate them.

**/


#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class LaserScanIntensityShorteningFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
public:

  double threshold_ ;
  double shortening_distance_ ;

  bool configure()
  {
    threshold_ = 1.0;
    shortening_distance_ = 0.25;
    getParam("threshold", threshold_);
    getParam("shortening_distance", shortening_distance_) ;

    return true;
  }

  virtual ~LaserScanIntensityShorteningFilter(){}

  bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
  {
    filtered_scan = input_scan;

    // Need to check ever reading in the current scan
    for (unsigned int i=0;
         i < input_scan.ranges.size() && i < input_scan.intensities.size();
         ++i)
    {
      // Normalize the intensity by the square of the range
      double scan_distance = input_scan.ranges[i];
      if (std::isfinite(scan_distance) && scan_distance >= input_scan.range_min)
      {
        // filtered_scan.intensities[i] *= std::sqrt(scan_distance) / (4.0);
        // Shorten anything above the threshold
        if (filtered_scan.intensities[i] > threshold_)
        {
          // If so, then make shorter
          filtered_scan.ranges[i] = std::max(filtered_scan.ranges[i] - shortening_distance_, 0.0);
        }
      }
    }

    return true;
  }
};
}

#endif // LASER_SCAN_INTENSITY_FILTER_H
