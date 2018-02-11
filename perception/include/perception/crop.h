#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"

namespace perception {
  class Cropper {
   public:
    Cropper();
    Cropper(const ros::Publisher& pub);
    void Callback(const sensor_msgs::PointCloud2& msg);

   private:
    ros::Publisher pub_;
  };
}