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

/**
\author Zach Fang
@brief A laserscan validator to check all range readings are reasonable,
       more specifically, not occluded or something very close to the 
       device.  
**/
#ifndef SCAN_VALIDATOR_h
#define SCAN_VALIDATOR_h

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters
{

class ScanValidator : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:

  ScanValidator();

  virtual ~ScanValidator();

  bool configure();

  bool update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan& filtered_scan);

  private:

  bool checkLaserConfig(
    const sensor_msgs::LaserScan& scan);

  ros::NodeHandle pnh_;

  /*
   * @brief occlusion percentage to determine a false laserscan
   */
  double lidar_occlusion_threshold_;

  /*
   * @brief invalid value percentage for warning about low-quality scans
   */
  double lidar_invalid_threshold_;

  /*
   * @brief Contour for the robot lidar shroud
   */
  std::vector<double> contour_;

  /*
   * @brief Tolerance for the contour distance measurement
   */
  double contour_tolerance_;

  /*
   * @brief Flag indicating whether the laser confiuration is checked
   */
  bool has_checked_laser_config_;

  double angle_min_;

  double angle_max_;

  double angle_increment_;

  double range_min_;

  double range_max_;
};

} // namespace laser_filters

#endif /* scan_validator.h */
