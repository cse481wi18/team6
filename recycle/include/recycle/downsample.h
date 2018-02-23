#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/filters/voxel_grid.h"

namespace perception {
  class Downsampler {
   public:
    Downsampler();
    PointCloudC::Ptr Callback(PointCloudC::Ptr cloud);
  };
}