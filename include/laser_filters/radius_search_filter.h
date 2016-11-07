/*
 * (c) Copyright 2015-2016 6 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 *
 * radius_search_filter.h
 *
 * created: 11/02/2016
 */



#ifndef RADIUS_SEARCH_FILTER_H
#define RADIUS_SEARCH_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace laser_filters
{
/**
 * @brief This is a filter that remove isolated points based on radius search.
 * If not enough neighbors are around, the point will be abandoned and its
 * range will be set to NaN.
 */
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
	double threshold_radius_;

	RadiusSearchFilter();

	virtual ~RadiusSearchFilter();

	bool configure();

    bool update(
      const sensor_msgs::LaserScan& input_scan,
      sensor_msgs::LaserScan& filtered_scan);
};

}

#endif /* radius_search_filter.h */
