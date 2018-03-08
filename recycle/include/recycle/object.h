#ifndef _RECYCLE_OBJECT_H_
#define _RECYCLE_OBJECT_H_

#include <string>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

namespace recycle {
struct Object {
  std::string ff_name;
  std::string pcf_name;
  
  std::string name;
  double confidence;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  geometry_msgs::Pose pose;
  geometry_msgs::Vector3 dimensions;
};
}  // namespace recycle

#endif  // _RECYCLE_OBJECT_H_