#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "recycle/classifier.h"
#include <ros/console.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "classifier");
  ros::NodeHandle nh;
  recycle::Classifier classifier("recycle_classifier/classify", "recycle_classifier/add_item");

  ros::spin();
  return 0;
}