#ifndef STEP_FILTER_H
#define STEP_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class StepFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    StepFilter();
    bool configure();

    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

  private:
    int num_total_beams_;

};
}
#endif // STEP_FILTER_H
