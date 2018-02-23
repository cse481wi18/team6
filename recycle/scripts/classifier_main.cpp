#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "recycle/classifier.h"
#include <ros/console.h>

int main(int argc, char** argv) {
  // if (argc < 2) {
  //   ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
  //   ros::spinOnce();
  // }
  // std::string data_dir(argv[1]);

  ros::init(argc, argv, "point_cloud_demo");
  ros::NodeHandle nh;

  // Cropper
  // ros::Publisher crop_pub =
  //     nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  recycle::Classifier classifier("recycle_classifier/classify");
  // ros::Subscriber point_cloud_sub =
  //     nh.subscribe("cloud_in", 1, &recycle::Classifier::PointCloudCallback, &point_cloud_sub);

  ros::spin();
  return 0;
}