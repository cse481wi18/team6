#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "recycle/classifier.h"
#include "recycle/crop.h"
#include "recycle/downsample.h"
#include "recycle/object_recognizer.h"
#include "recycle/segmentation.h"

namespace recycle {
  Classifier::Classifier(std::string name) :
    as_(nh_, name, boost::bind(&Classifier::ActionCallback, this, _1), false),
    action_name_(name) {
      as_.start();
  }

  void Classifier::ActionCallback(const recycle_msgs::ClassifyGoalConstPtr &goal)  {
    recycle_msgs::ClassifyResult result;

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

    ROS_INFO("OBJECT RECOGNIZER");
    // This SHOULD be perception_msgs and not recycle_msgs
    std::vector<perception_msgs::ObjectFeatures> dataset;
    // TODO: LOAD DATA FROM DATABASE
    recycle::LoadData("/home/team6/data/objects/combined_labels/", &dataset);
    recycle::ObjectRecognizer recognizer(dataset);
    ROS_INFO("LOADED DATASET");

    ROS_INFO("SEGMENGTING");
    ros::NodeHandle nh;
    ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    
    recycle::Segmenter segmenter(table_pub, recognizer);
    segmenter.SegmentAndClassify(downsampled, &result);
    ROS_INFO("SEGMENTED");

    ROS_INFO("REPLYING TO CLIENT");
    as_.setSucceeded(result);
    ROS_INFO("REPLIED");
  } 
}
