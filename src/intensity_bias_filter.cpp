#include <algorithm>
#include "laser_filters/intensity_bias_filter.h"
#include <ros/ros.h>

laser_filters::IntensityBiasFilter::IntensityBiasFilter() {

}

bool laser_filters::IntensityBiasFilter::configure() {
  bool total_beams_set = getParam("total_beams", total_beams_);
  bool intensity_thresh_set = getParam("intensity_threshold", intensity_threshold_);
  bool max_instensity_selected_set = getParam("max_intensity_selected", max_intensity_selected_);

  if (max_instensity_selected_set > total_beams_set) {
      ROS_WARN("Maximum beams chosen by intensity is greater than total beams");
  }

  return total_beams_set && intensity_thresh_set && max_instensity_selected_set;
}

bool laser_filters::IntensityBiasFilter::update(
    const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
    // all of the laser filters do this to save processing?
    scan_out = scan_in;
    int total_num_intensity_beams = std::count_if(scan_out.intensities.begin(), scan_out.intensities.end(), [&](int i){return i > intensity_threshold_; });
    int select_num_intensity_beams = std::min(total_num_intensity_beams, max_intensity_selected_);
    int num_uniform_beams = total_beams_ - select_num_intensity_beams;

    // calculate step sizes for uniform sampling and intensity sampling
    int uniform_step_size = scan_in.ranges.size() / num_uniform_beams;
    int intensity_step_size = scan_in.ranges.size() / select_num_intensity_beams ;
    // NaN anything we don't want
    int intensity_count = 0;
    for (unsigned int i=0; i < scan_out.ranges.size(); i++) {
      // does this beam qualify for uniform sampling
      bool keep_uniform = (i % uniform_step_size);

      // does this beam qualify for intensity sampling
      if (scan_out.intensities[i] > intensity_threshold_) intensity_count++;
      bool keep_intensity = (intensity_count % intensity_step_size);
      
      bool keep_beam = keep_uniform || keep_intensity;
      if (!keep_beam) {
        scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      }
    }
    return true;
}