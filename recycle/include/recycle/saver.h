#include <vector>

#include "recycle/object.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "pcl/common/common.h"
#include "pcl/ModelCoefficients.h"
#include "recycle_msgs/ObjectFeatures.h"
#include "recycle_msgs/LogItem.h"

namespace recycle {
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

class Saver {
 public:
  Saver(const ros::Publisher& table_cloud_pub);
	int Saver::PredictAndLog(std::string classification, PointC cloud
          recycle::ObjectFeatures* features);
	int Saver::ActualAndLog(std::string classification, PointC cloud
          recycle::ObjectFeatures* features);

 private:
  ros::Publisher logger_pub_;
	int SaveAndLog(std::string classification, pcl::PointXYZRGB PointC
					recycle::ObjectFeatures* features);
 };
}  // namespace recycle
