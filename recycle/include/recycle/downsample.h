#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

namespace recycle {
  class Downsampler {
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
    public:
      Downsampler(ros::Publisher pub);
      PointCloudC::Ptr Downsample(PointCloudC::Ptr cloud);
    private:
      ros::Publisher pub_;
  };
}