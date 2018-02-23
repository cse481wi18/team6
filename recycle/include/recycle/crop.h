#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"

namespace recycle {
  typedef pcl::PointXYZRGB PointC;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
  class Cropper {
   public:
    Cropper();
    PointCloudC::Ptr Crop(const PointCloudC::Ptr cloud);
  };
}