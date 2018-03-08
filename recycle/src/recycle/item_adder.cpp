#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "recycle/item_adder.h"
#include "recycle/crop.h"
#include "recycle/downsample.h"
#include "recycle/object_recognizer.h"
// #include "recycle/saver.h"
#include "recycle/segmentation.h"
#include "recycle_msgs/LogItem.h"

namespace recycle {
  ItemAdder::ItemAdder(std::string name) :
    as_(nh_, name, boost::bind(&ItemAdder::ActionCallback, this, _1), false),
    action_name_(name) {
      as_.start();
  }

 void ItemAdder::ActionCallback(const recycle_msgs::AddItemGoalConstPtr &goal)  {
    //Create result message
    recycle_msgs::AddItemResult result;

    ROS_INFO("WAITING FOR MESSAGE");
    
    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");
    
    ROS_INFO("RECEIVED MESSAGE");
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(* msg, *cloud);

    ROS_INFO("TRYING TO CROP");
    
    Cropper cropper;
    PointCloudC::Ptr cropped = cropper.Crop(cloud);
    
    ROS_INFO("CROPPED SUCCESSFULLY");

    ROS_INFO("TRYING TO DOWNSAMPLE");
    Downsampler downsampler;
    PointCloudC::Ptr downsampled = downsampler.Downsample(cropped);
    ROS_INFO("DOWNSAMPLED SUCCESSFULLY");

    recycle::ObjectRecognizer dummy;

    ROS_INFO("SEGMENGTING");
    ros::NodeHandle nh;
    ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    
    recycle::Segmenter segmenter(table_pub, dummy);

    ROS_INFO("SEGMENTED");

    ROS_INFO("REPLYING TO CLIENT");
    as_.setSucceeded(result);
    ROS_INFO("REPLIED");
  } 
}