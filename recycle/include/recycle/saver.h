#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "rosbag/bag.h"

#include "pcl_ros/transforms.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/common.h"
#include "pcl/ModelCoefficients.h"

#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"

#include "recycle/object.h"
#include "recycle_msgs/ObjectFeatures.h"
#include "recycle_msgs/LogItem.h"

namespace recycle {
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

class Saver {
 public:
 	Saver();
	void Log(Object object, 
		recycle_msgs::ObjectFeatures* features, 
		recycle_msgs::LogItem*,
		bool prediction);

 private:
  static const std::string location;
  static const std::string label_sub_directory;
	int SaveFiles(Object object, 
		recycle_msgs::ObjectFeatures* features,
		std::string point_cloud_path,
		std::string features_path);
 };
}  // namespace recycle
