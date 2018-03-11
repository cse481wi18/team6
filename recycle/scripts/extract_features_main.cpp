#include <iostream>
#include <string>
#include <vector>

#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"

#include "recycle/feature_extraction.h"
#include "recycle/object.h"
#include "recycle/segmentation.h"
#include "recycle_msgs/ObjectFeatures.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void Crop(PointCloudC::Ptr cloud_in, PointCloudC::Ptr cloud_out) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  ros::param::param("crop_min_x", min_x, 0.3);
  ros::param::param("crop_min_y", min_y, -1.0);
  ros::param::param("crop_min_z", min_z, 0.5);
  ros::param::param("crop_max_x", max_x, 0.9);
  ros::param::param("crop_max_y", max_y, 1.0);
  ros::param::param("crop_max_z", max_z, 1.5);
  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud_in);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.filter(*cloud_out);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_features");
  ros::Time::init();
  if (argc < 3) {
    std::cout << "Extracts features from a bag file with a point cloud with a "
                 "single object on a table. The features are saved to LABEL.bag"
              << std::endl;
    std::cout << "Usage: rosrun recycle extract_features FILE.bag LABEL"
              << std::endl;
    return 0;
  }
  std::string path(argv[1]);
  std::string label(argv[2]);
  std::cout << path << std::endl;
  std::cout << label << std::endl;
  // ROS_INFO(path);
  // ROS_INFO(label);
  rosbag::Bag bag;
  bag.open("/home/team6/data/demo_objects/" + path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("head_camera/depth_registered/points");
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  if (view.size() == 0) {
    std::cerr << "No messages on topic head_camera/depth_registered/points"
              << std::endl;
  }

  PointCloudC::Ptr pcl_cloud(new PointCloudC());
  for (rosbag::View::const_iterator it = view.begin(); it != view.end(); ++it) {
    sensor_msgs::PointCloud2::ConstPtr cloud =
        it->instantiate<sensor_msgs::PointCloud2>();
    if (cloud == NULL) {
      std::cerr << "Unable to instantiate point cloud." << std::endl;
      return 1;
    }
    pcl::fromROSMsg(*cloud, *pcl_cloud);
    break;
  }

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  Crop(pcl_cloud, cropped_cloud);

  std::vector<recycle::Object> objects;
  std::vector<recycle::Object> obstacles;
  recycle::ObjectRecognizer dummy_recognizer;

  PointCloudC::Ptr above_surface_cloud(new PointCloudC);
  ros::NodeHandle nh;

  // Cropper
  ros::Publisher table =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
  ros::Publisher above =
      nh.advertise<sensor_msgs::PointCloud2>("above_cloud", 1, true);
  recycle::Segmenter segmenter(table, above, dummy_recognizer);
  segmenter.SegmentTabletopScene(cropped_cloud, &objects, &obstacles, above_surface_cloud, true);

  if (objects.size() != 1) {
    std::cerr << path << " Expected to see exactly one object, found " << objects.size()
              << std::endl;
    // for (int i = 0; i < objects.size(); i++) {
    //   std::cerr << "Object " << i << " " << objects[0].dimensions
    //             << std::endl;
      
    // }
    return 1;
  }

  const recycle::Object& object = objects[0];
  recycle_msgs::ObjectFeatures features;
  features.classification = label;
  recycle::ExtractFeatures(object, &features);

  rosbag::Bag bag_out;
  bag_out.open("/home/team6/data/demo_objects/" + label + "_label.bag", rosbag::bagmode::Write);
  bag_out.write("object_features", ros::Time::now(), features);
  bag_out.close();

  return 0;
}
