#ifndef _RECYCLE_ITEM_ADDER_H_
#define _RECYCLE_ITEM_ADDER_H_

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
#include "recycle_msgs/AddItemAction.h"

namespace recycle {
  typedef pcl::PointXYZRGB PointC;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
  
  class ItemAdder {
    public:
     explicit ItemAdder(std::string name);  
     void ActionCallback(const recycle_msgs::AddItemGoalConstPtr &goal);

    private:
      ros::NodeHandle nh_;
      actionlib::SimpleActionServer<recycle_msgs::AddItemAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
      std::string action_name_;
      // sensor_msgs::PointCloud2* pcloud_;
  };
}


#endif