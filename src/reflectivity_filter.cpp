#include "laser_filters/reflectivity_filter.h"
#include <ros/ros.h>

laser_filters::ReflectivityFilter::ReflectivityFilter() {

}

bool laser_filters::ReflectivityFilter::configure() {
  bool reflectivity_thresh_set = getParam("reflectivity_threshold", reflectivity_thresh_);
  bool distance_scale_set = getParam("distance_scale", distance_scale_);

  return reflectivity_thresh_set && distance_scale_set;
}

bool laser_filters::ReflectivityFilter::update(
    const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
    // all of the laser filters do this to save processing?
    scan_out = scan_in;

    for (unsigned int i=0; i < scan_out.ranges.size(); i++) {
        // check if intensity * (distance_scale / range) > reflectivity_thresh
        if (scan_out.intensities[i] * distance_scale_ * (1/scan_out.ranges[i]) < reflectivity_thresh_) {
            scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    return true;
}