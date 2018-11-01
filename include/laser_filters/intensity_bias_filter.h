#ifndef INTENSITY_BIAS_FILTER_H
#define INTENSITY_BIAS_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class IntensityBiasFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    IntensityBiasFilter();
    bool configure();

    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

  private:
    int num_total_beams_;
    float high_intensity_threshold_;
    int max_num_high_intensity_beams_;


};
}
#endif // INTENSITY_BIAS_FILTER_H