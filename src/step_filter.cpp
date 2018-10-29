#include "laser_filters/step_filter.h"
#include <ros/ros.h>

laser_filters::StepFilter::StepFilter() {

}

bool laser_filters::StepFilter::configure() {
  bool num_total_beams_set = getParam("num_total_beams", num_total_beams_);

  return num_total_beams_set;
}

bool laser_filters::StepFilter::update(
    const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
    // all of the laser filters do this to save processing?
    scan_out = scan_in;
    
    num_total_beams_ = std::max(num_total_beams_, 1);
    double beam_step_size = (double) scan_out.ranges.size() / num_total_beams_;
    double beam_step = 0;
    for (unsigned int i=0; i < scan_out.ranges.size(); i++) {
        bool keep_beam = false;
        if ( (int) beam_step == i) {
            keep_beam = true;
            beam_step += beam_step_size;
        }

        // NaN out any beams that are NOT in the sample
        if (!keep_beam) {
            scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    return true;
}