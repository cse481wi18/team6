#include "recycle/saver.h"

namespace recycle {

const std::string Saver::location = "/home/team6/varun_temp/";
const std::string Saver::label_sub_directory = "temp_labels/";

Saver::Saver() {}

void Saver::Log(Object object, 
                recycle_msgs::ObjectFeatures* features, 
                recycle_msgs::LogItem* item,
                bool prediction) {
  item->log_id = -1;
  std::string classification(object.name);
  if (prediction) {
    item->predicted_category = classification;
  } else {
    item->actual_category = classification;
  }
  ROS_INFO_STREAM("Time is " << ros::Time::now().toSec());
  

  std::stringstream point_cloud_path;
  std::stringstream features_path;

  // point_cloud_path << location << classification << ros::Time::now().toSec() << ".bag";
  // features_path << location << label_sub_directory << classification << ros::Time::now().toSec() << "_label.bag";

  int num = rand();

  point_cloud_path << location << classification << num << ".bag";
  features_path << location << label_sub_directory << num << "_label.bag";


  std::string pc_path = point_cloud_path.str();
  std::string f_path = features_path.str();

  item->pointcloud_file_path = pc_path;
  item->feature_file_path = f_path;
  SaveFiles(object, features, pc_path, f_path);
}

int Saver::SaveFiles(Object object, 
                      recycle_msgs::ObjectFeatures* features, 
                      std::string pc_path,
                      std::string f_path) {

  // pcl_ros::transformPointCloud("base_link", transform, *cloud, cloud_out);

  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
  sensor_msgs::PointCloud2 save_cloud_msg;
  pcl::toROSMsg(*(object.cloud), save_cloud_msg);

  tf::TransformListener tf_listener;                                                    
  tf_listener.waitForTransform("base_link", save_cloud_msg.header.frame_id,                     
                               ros::Time(0), ros::Duration(5.0));                       
  tf::StampedTransform transform;                                                       
  try {                                                                                 
    tf_listener.lookupTransform("base_link", save_cloud_msg.header.frame_id,                    
                                ros::Time(0), transform);                               
  } catch (tf::LookupException& e) {                                                    
    std::cerr << e.what() << std::endl;                                                 
    return 1;                                                                           
  } catch (tf::ExtrapolationException& e) {                                             
    std::cerr << e.what() << std::endl;                                                 
    return 1;                                                                           
  }                                                                                     
                                                                                          
  sensor_msgs::PointCloud2 cloud_out;                                                   
  pcl_ros::transformPointCloud("base_link", transform, save_cloud_msg, cloud_out);

  rosbag::Bag bag_out;
  bag_out.open(pc_path, rosbag::bagmode::Write);
  bag_out.write("head_camera/depth_registered/points", ros::Time::now(), cloud_out);
  bag_out.close();

  ROS_INFO_STREAM("SAVING FEATURE FILE " << f_path);
  rosbag::Bag bag_out2;
  bag_out2.open(f_path, rosbag::bagmode::Write);
  bag_out2.write("object_features", ros::Time::now(), *features);
  bag_out2.close();
}

}  // namespace recycle