#include "recycle/downsample.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/common/common.h"

namespace recycle {
  typedef pcl::PointXYZRGB PointC;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

  Downsampler::Downsampler(ros::Publisher pub): pub_(pub) {}

  PointCloudC::Ptr Downsampler::Downsample(PointCloudC::Ptr cloud) {
    ROS_INFO("Got point cloud with %ld points", cloud->size());

    PointCloudC::Ptr downsampled_cloud(new PointCloudC());
    pcl::VoxelGrid<PointC> vox;
    vox.setInputCloud(cloud);
    double voxel_size;
    ros::param::param("voxel_size", voxel_size, 0.01);
    vox.setLeafSize(voxel_size, voxel_size, voxel_size);
    vox.filter(*downsampled_cloud);

    sensor_msgs::PointCloud2 msg_out;
    pcl::toROSMsg(*downsampled_cloud, msg_out);
    pub_.publish(msg_out);

    ROS_INFO("Downsampled to %ld points", downsampled_cloud->size());
    return downsampled_cloud;
  }
}