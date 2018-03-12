// TODO: add includes, etc.
#include "recycle/crop.h"

namespace recycle {
  Cropper::Cropper() {}
  Cropper::Cropper(const ros::Publisher& pub) :pub_(pub){}
  PointCloudC::Ptr Cropper::Crop(const PointCloudC::Ptr cloud, bool classify) {
    
    ROS_INFO("Got point cloud with %ld points", cloud->size());
    

    double min_x, min_y, min_z, max_x, max_y, max_z;

    if (classify) {
        ros::param::param("crop_min_x", min_x, 0.3);
        ros::param::param("crop_min_y", min_y, -1.0);
        ros::param::param("crop_min_z", min_z, 0.5);

        ros::param::param("crop_max_x", max_x, 0.9);
        ros::param::param("crop_max_y", max_y, 1.0);
        ros::param::param("crop_max_z", max_z, 1.5);
    } else {
        ros::param::param("crop_min_x_add", min_x, 0.3);
        ros::param::param("crop_min_y_add", min_y, -1.0);
        ros::param::param("crop_min_z_add", min_z, 0.5);

        ros::param::param("crop_max_x_add", max_x, 0.9);
        ros::param::param("crop_max_y_add", max_y, 1.0);
        ros::param::param("crop_max_z_add", max_z, 1.5);
    }
    
    Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
    Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
    pcl::CropBox<PointC> crop;
    crop.setInputCloud(cloud);
    crop.setMin(min_pt);
    crop.setMax(max_pt);
    
    PointCloudC::Ptr cropped_cloud(new PointCloudC());
    crop.filter(*cropped_cloud);
    ROS_INFO("Cropped to %ld points", cropped_cloud->size());

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*cloud, msg_out);
    pub_.publish(msg_out);
    return cropped_cloud;
  }
}