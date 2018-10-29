#include <algorithm>
#include "laser_filters/intensity_bias_filter.h"
#include <ros/ros.h>

laser_filters::IntensityBiasFilter::IntensityBiasFilter() {

}

bool laser_filters::IntensityBiasFilter::configure() {
  bool num_total_beams_set = getParam("num_total_beams", num_total_beams_);
  bool high_intensity_thresh_set = getParam("intensity_threshold", high_intensity_threshold_);
  bool max_num_high_intensity_beams_set = getParam("max_intensity_selected", max_num_high_intensity_beams_);

  if (max_num_high_intensity_beams_ > num_total_beams_) {
      ROS_WARN("Maximum beams chosen by intensity is greater than total beams");
  }

  return num_total_beams_set && high_intensity_thresh_set && max_num_high_intensity_beams_set;
}

bool laser_filters::IntensityBiasFilter::update(
    const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
    // all of the laser filters do this to save processing?
    scan_out = scan_in;

    // calculate how many high intensity beams to select
    int total_num_high_intensity_beams = std::count_if(scan_out.intensities.begin(), scan_out.intensities.end(), [&](int i){return i > high_intensity_threshold_; });
    int num_high_intensity_beams = std::min(total_num_high_intensity_beams, max_num_high_intensity_beams_);
    int num_uniform_beams = num_total_beams_ - num_high_intensity_beams;

    // calculate step sizes for uniform sampling and intensity sampling
    // we don't want to divide by zero or anything negative
    // we also don't want step size to be less than 1
    num_uniform_beams = std::max(num_uniform_beams, 1);
    num_high_intensity_beams = std::max(num_high_intensity_beams, 1);
    int uniform_step_size = std::max( (int) scan_in.ranges.size() / num_uniform_beams, 1);
    int intensity_step_size = std::max(total_num_high_intensity_beams / num_high_intensity_beams, 1);
     
    // NaN anything we don't want
    int intensity_count = 0;
    for (unsigned int i=0; i < scan_out.ranges.size(); i++) {
      // does this beam qualify for uniform sampling
      bool keep_uniform = !(i % uniform_step_size);

      // does this beam qualify for intensity sampling
      bool keep_intensity = false;
      if (scan_out.intensities[i] > high_intensity_threshold_) {
        intensity_count++;
        keep_intensity = !(intensity_count % intensity_step_size);
      }

      bool keep_beam = keep_uniform || keep_intensity;
      if (!keep_beam) {
        scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    return true;
}