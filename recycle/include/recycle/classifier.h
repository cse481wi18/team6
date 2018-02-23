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

#include "sensor_msgs/PointCloud2.h"
#include "recycle_msgs/ClassifyAction.h"

namespace recycle {
  typedef pcl::PointXYZRGB PointC;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
  
  class Classifier {
    public:
     explicit Classifier(std::string name);  
     void ActionCallback(const recycle_msgs::ClassifyGoalConstPtr &goal);
     // void PointCloudCallback(const sensor_msgs::PointCloud2& msg);

    private:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<recycle_msgs::ClassifyAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string action_name_;
      // sensor_msgs::PointCloud2* pcloud_;
  };
}


#endif