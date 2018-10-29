#include <algorithm>
#include "laser_filters/intensity_bias_filter.h"
#include <ros/ros.h>

laser_filters::IntensityBiasFilter::IntensityBiasFilter() {

}

bool laser_filters::IntensityBiasFilter::configure() {
  bool num_total_beams_set = getParam("num_total_beams", num_total_beams_);
  int high_intensity_threshold_raw_param = 0;
  bool high_intensity_thresh_set = getParam("intensity_threshold", high_intensity_threshold_raw_param);
  high_intensity_threshold_ = static_cast<float>(high_intensity_threshold_raw_param);
  bool max_num_high_intensity_beams_set = getParam("max_num_high_intensity_beams", max_num_high_intensity_beams_);
  if (max_num_high_intensity_beams_ > num_total_beams_) {
      ROS_WARN_STREAM("Maximum beams chosen by intensity (" << max_num_high_intensity_beams_ <<") is greater than total beams(" << num_total_beams_ <<")");
  }

  return num_total_beams_set && high_intensity_thresh_set && max_num_high_intensity_beams_set;
}

bool laser_filters::IntensityBiasFilter::update(
    const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out)
{
    // all of the laser filters do this to save processing?
    scan_out = scan_in;

    // calculate how many high intensity beams to select
    int total_num_high_intensity_beams = std::count_if(scan_out.intensities.begin(), scan_out.intensities.end(), [&](float i){return i > high_intensity_threshold_; });
    int num_high_intensity_beams = std::min(total_num_high_intensity_beams, max_num_high_intensity_beams_);
    int num_uniform_beams = num_total_beams_ - num_high_intensity_beams;
    ROS_DEBUG_STREAM("Found " << total_num_high_intensity_beams << " total high intensity beams ( > " << high_intensity_threshold_ << "). Choosing " << num_high_intensity_beams << " high intensity and " << num_uniform_beams << " uniform beams.");
    
    // calculate step sizes for uniform sampling and intensity sampling
    // we don't want to divide by zero or anything negative
    // we also don't want step size to be less than 1
    num_uniform_beams = std::max(num_uniform_beams, 1);
    num_high_intensity_beams = std::max(num_high_intensity_beams, 1);
    double uniform_step_size = static_cast<double>(scan_in.ranges.size()) / num_uniform_beams;
    double intensity_step_size = static_cast<double>(total_num_high_intensity_beams) / num_high_intensity_beams;
    ROS_DEBUG_STREAM("Filtering beam for " << num_uniform_beams << " uniform beams and " << num_high_intensity_beams << " high intensity beams");
    ROS_DEBUG_STREAM("Uniform step size: " << uniform_step_size << " High intensity step size: " << intensity_step_size);
    
    // NaN anything we don't want
    int intensity_count = 0;
    double uniform_step = 0;
    double intensity_step = 0;
    int kept_beams = 0;
    for (unsigned int i=0; i < scan_out.ranges.size(); i++) {
      // does this beam qualify for uniform sampling
      bool keep_uniform = false;

      if (static_cast<int>(uniform_step) == i) {
        keep_uniform = true;
        ROS_DEBUG_STREAM_NAMED("uniform_beams", "Uniform sampling keeping beam " << i << "with uniform step "<<uniform_step);
        uniform_step += uniform_step_size;
      }

      // does this beam qualify for intensity sampling
      bool keep_intensity = false;
      if (scan_out.intensities[i] > high_intensity_threshold_) {
        if (static_cast<int>(intensity_step) == intensity_count) {
          keep_intensity = true;
          ROS_DEBUG_STREAM_NAMED("intensity_beams", "Intensity sampling keeping beam " << i << "with intensity step "<<intensity_step);
          intensity_step += intensity_step_size;
        }
        intensity_count++;
      }

      bool keep_beam = keep_uniform || keep_intensity;
      if (!keep_beam) {
        scan_out.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      } else {
        kept_beams += 1;
      }
    }

    ROS_DEBUG_STREAM("Kept " << kept_beams << " beams");
    return true;
}