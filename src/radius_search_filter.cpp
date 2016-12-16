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
#include "laser_filters/radius_search_filter.h"
#include <ros/ros.h>

laser_filters::RadiusSearchFilter::RadiusSearchFilter(){

}

laser_filters::RadiusSearchFilter::~RadiusSearchFilter(){

}

bool laser_filters::RadiusSearchFilter::configure()
{
    // Setup default values
    neighbor_num_ = 3;
    threshold_num_ = 3;
    threshold_radius_ = 0.20;

    // launch values from parameter server
    getParam("neighbor_number", neighbor_num_);
    getParam("threshold_number", threshold_num_);
    getParam("threshold_radius",  threshold_radius_);

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
		if(output_scan.ranges[i] == std::numeric_limits<float>::quiet_NaN())
		{
			continue;
		}

		int counter = 0;
		// for each point, traverse its neighbors
		for(int j= -neighbor_num_; j< neighbor_num_ + 1; j++)
		{
			int neighbor_index = i + j;

			// skip out-of-bound points and the point itself
			if( neighbor_index< 0 || neighbor_index >= (int)input_scan.ranges.size() || neighbor_index == i)
			{
				continue;
			}

			if(fabs(input_scan.ranges[i]- input_scan.ranges[neighbor_index]) <= threshold_radius_)
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

