#include "laser_filters/step_filter.h"
#include <ros/ros.h>

laser_filters::StepFilter::StepFilter() {

}

bool laser_filters::StepFilter::configure() {
  bool num_beams_set = getParam("num_beams", num_beams_);

  return num_beams_set;
}

bool laser_filters::StepFilter::update(
    const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
    // all of the laser filters do this to save processing?
    scan_out = scan_in;
    int beam_step = scan_out.ranges.size() / num_beams_;
    for (unsigned int i=0; i < scan_out.ranges.size(); i++) {
        // NaN out any beams that are NOT in the sample
        if (!!(i % beam_step)) {
            scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    return true;
}