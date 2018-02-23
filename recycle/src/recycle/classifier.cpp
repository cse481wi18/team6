#include "recycle/classifier.h"
#include "recycle/crop.h"
#include "recycle/downsample.h"

namespace recycle {
  Classifier::Classifier(std::string name) :
    as_(nh_, name, boost::bind(&Classifier::ActionCallback, this, _1), false),
    action_name_(name) {
      as_.start();
  }

  // void PointCloudCallback(const sensor_msgs::PointCloud2& msg) {
  //   pcloud_ = msg;
  // }

  void Classifier::ActionCallback(const recycle_msgs::ClassifyGoalConstPtr &goal)  {
    recycle_msgs::ClassifyResult result;
    ROS_INFO("WAITING FOR MESSAGE");
    
    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");
    
    ROS_INFO("RECEIVED MESSAGE");
    sensor_msgs::PointCloud2 pc;
    PointCloudC::Ptr cloud(new PointCloudC());
    pc = * msg;
    pcl::fromROSMsg(pc, *cloud);


    ROS_INFO("TRYING TO CROP");
    
    Cropper cropper;
    PointCloudC::Ptr cropped = cropper.Crop(cloud);
    
    ROS_INFO("CROPPED SUCCESSFULLY");

    ROS_INFO("TRYING TO DOWNSAMPLE");
    Downsampler downsampler;
    PointCloudC::Ptr downsampled = cropper.Crop(cropped);
    ROS_INFO("DOWNSAMPLED SUCCESSFULLY");

    result.num_objects = 3;
    result.classifications.push_back("coffee_cup_no_sleeve");    
    result.classifications.push_back("crumpled_paper");
    result.classifications.push_back("nature_valley_wrapper");

  } 
}
