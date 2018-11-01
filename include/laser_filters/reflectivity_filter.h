#ifndef REFLECTIVITY_FILTER_H
#define REFLECTIVITY_FILTER_H

#include "filters/filter_base.h"
#include "sensor_msgs/LaserScan.h"

namespace laser_filters
{

class ReflectivityFilter : public filters::FilterBase<sensor_msgs::LaserScan>
{
  public:
    ReflectivityFilter();
    bool configure();

    bool update(const sensor_msgs::LaserScan& scan_in, sensor_msgs::LaserScan& scan_out);

  private:
    double reflectivity_thresh_;
    double distance_scale_;

};
}
#endif // REFLECTIVITY_FILTER_H
