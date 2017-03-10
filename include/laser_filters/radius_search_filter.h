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
@brief This is a filter that remove isolated points based on radius search. If not enough neighbors are around, the point will be abandoned and its range will be set to NaN.
**/


#ifndef RADIUS_SEARCH_FILTER_H
#define RADIUS_SEARCH_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <laser_filters/LaserConfig.h>

namespace laser_filters
{

class RadiusSearchFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:

  /*
   * @brief The number of candidates each side to consider
   */
  int neighbor_num_;

  /*
   * @brief The minimum amount of neighbors a point should have to be
   * considered as a valid point
   */
  int threshold_num_;

  /*
   * @brief The distance to be neighbor
   */
  double threshold_ratio_;

  RadiusSearchFilter();

  virtual ~RadiusSearchFilter();

  bool configure();

  bool update(
    const sensor_msgs::LaserScan& input_scan,
    sensor_msgs::LaserScan& filtered_scan);

  private:

    void reconfigureCB(laser_filters::LaserConfig& config, uint32_t level);

};

}

#endif /* radius_search_filter.h */
