#ifndef _RECYCLE_CLASSIFIER_H_
#define _RECYCLE_CLASSIFIER_H_

#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib/server/simple_action_server.h>
#include "pcl/common/common.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "recycle_msgs/ClassifyAction.h"
#include "recycle_msgs/AddItemAction.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"

#include "recycle/crop.h"
#include "recycle/downsample.h"
#include "recycle/object_recognizer.h"
#include "recycle/segmentation.h"

namespace recycle {
  typedef pcl::PointXYZRGB PointC;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
  
  class Classifier {
    public:
     explicit Classifier(std::string database_path,
                         std::string classify_name, 
                         std::string add_item_name, 
                         std::string logger_name);  

    private:
      ros::NodeHandle nh_;
      ros::Publisher crop_pub_;
      ros::Publisher above_table_pub_;
      std::string database_path_;
      std::string classifier_action_name_;
      std::string add_item_action_name_;
      std::string logger_name_;
      actionlib::SimpleActionServer<recycle_msgs::ClassifyAction> classifier_as_;
      actionlib::SimpleActionServer<recycle_msgs::AddItemAction> add_item_as_;
      actionlib::SimpleActionClient<recycle_msgs::DbLogAction> logging_ac_;
      void ClassifierActionCallback(const recycle_msgs::ClassifyGoalConstPtr &goal);
      void AddItemActionCallback(const recycle_msgs::AddItemGoalConstPtr &goal);
  };
}


#endif