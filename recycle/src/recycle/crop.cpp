// TODO: add includes, etc.
#include "recycle/crop.h"

namespace recycle {
  Cropper::Cropper() {}

  PointCloudC::Ptr Cropper::Crop(const PointCloudC::Ptr cloud) {
    
    ROS_INFO("Got point cloud with %ld points", cloud->size());

    PointCloudC::Ptr cropped_cloud(new PointCloudC());
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
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    crop.filter(*cropped_cloud);
    ROS_INFO("Cropped to %ld points", cropped_cloud->size());

    return cropped_cloud;
  }
}