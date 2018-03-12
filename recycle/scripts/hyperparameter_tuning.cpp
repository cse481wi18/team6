#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "recycle/classifier.h"
#include "recycle/object_recognizer.h"
#include <ros/console.h>
#include "recycle_msgs/ClassifyAction.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

int main(int argc, char** argv) {
  ros::init(argc, argv, "classifier");

  sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");

  tf::TransformListener tf_listener;
  tf_listener.waitForTransform("base_link", msg->header.frame_id,
                               ros::Time(0), ros::Duration(5.0));
  tf::StampedTransform transform;
  try {
    tf_listener.lookupTransform("base_link", msg->header.frame_id,
                                ros::Time(0), transform);
  } catch (tf::LookupException& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  } catch (tf::ExtrapolationException& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  sensor_msgs::PointCloud2 cloud_out;
  pcl_ros::transformPointCloud("base_link", transform, *msg, cloud_out);

  PointCloudC::Ptr cloud(new PointCloudC());
  pcl::fromROSMsg(cloud_out, *cloud);
  ros::NodeHandle nh;
  ros::Publisher crop_pub_ = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
  ros::Publisher crop_pub_2 = nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud2", 1, true);
  recycle::ObjectRecognizer rec;
  rec.LoadData("/home/team6/catkin_ws/src/cse481wi18/database/recycle.db");
  recycle::Segmenter segmenter(crop_pub_2, crop_pub_, rec);
  recycle_msgs::ClassifyResult classifications;
  segmenter.ClassifyCloud(cloud, &classifications);



  ros::spin();
  return 0;
}
