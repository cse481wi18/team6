#include "recycle/saver.h"
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"

namespace recycle {

Saver::Saver(const ros::Publisher& logger_pub,
                      const ObjectRecognizer& recognizer)
    : logger_pub_(logger_pub) {}

int Saver::PredictAndLog(std::string classification, PointC cloud
          recycle::ObjectFeatures* features) {
  recycle_msgs::LogItem item;
  item.predicted_category = classification;

  std::string location = "/home/team6/varun_temp/";
  std::string point_cloud_path = location + classification + "_cloud_" + ros::Time::now().toSec() + ".bag";
  std::string features_path = location + classification + "_features_" + ros::Time::now().toSec() + ".bag"

  item.pointcloud_file_path = point_cloud_path;
  item.feature_file_path = features_path;
  logger_pub.publish(item);
}

int Saver::ActualAndLog(std::string classification, PointC cloud
          recycle::ObjectFeatures* features) {
  recycle_msgs::LogItem item;
  item.actual_category = classification;

  std::string location = "/home/team6/varun_temp/";
  std::string point_cloud_path = location + classification + "_cloud_" + ros::Time::now().toSec() + ".bag";
  std::string features_path = location + classification + "_features_" + ros::Time::now().toSec() + ".bag"

  item.pointcloud_file_path = point_cloud_path;
  item.feature_file_path = features_path;
  logger_pub.publish(item);


}

int Saver::SaveAndLog(std::string classification, PointC cloud
          recycle::ObjectFeatures* features) {
  

  pcl_ros::transformPointCloud("base_link", transform, *cloud, cloud_out);

  rosbag::Bag bag_out;
  bag_out.open(point_cloud_path, rosbag::bagmode::Write);
  bag_out.write("head_camera/depth_registered/points", ros::Time::now(), cloud);
  bag_out.close();

  rosbag::Bag bag_out2;
  bag_out2.open(features_path, rosbag::bagmode::Write);
  bag_out2.write("object_features", ros::Time::now(), features);
  bag_out2.close();
}

}  // namespace recycle