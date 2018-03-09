#include "recycle/classifier.h"

namespace recycle {
  Classifier::Classifier(std::string classify_name,
      std::string add_item_name) :
    classifier_as_(nh_, classify_name, boost::bind(&Classifier::ClassifierActionCallback, this, _1), false),
    add_item_as_(nh_, add_item_name, boost::bind(&Classifier::AddItemActionCallback, this, _1), false),
    classifier_action_name_(classify_name),
    add_item_action_name_(add_item_name) {
      classifier_as_.start();
      add_item_as_.start();
    }

  void Classifier::ClassifierActionCallback(const recycle_msgs::ClassifyGoalConstPtr &goal)  {
    recycle_msgs::ClassifyResult result;

    ROS_INFO("CLASSIFYING");
    
    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");
    
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(* msg, *cloud);

    Cropper cropper;
    PointCloudC::Ptr cropped = cropper.Crop(cloud);
    Downsampler downsampler;
    PointCloudC::Ptr downsampled = downsampler.Downsample(cropped);

    recycle::ObjectRecognizer recognizer;
    // TODO: CHANGE THE DATABASE
    recognizer.LoadData("/home/team6/catkin_ws/src/cse481wi18/database/temp.db");

    ros::NodeHandle nh;
    ros::Publisher table_pub =
      nh.advertise<sensor_msgs::PointCloud2>("table_cloud", 1, true);
    ROS_INFO("Segmenting");
    recycle::Segmenter segmenter(table_pub, recognizer);
    segmenter.SegmentAndClassify(downsampled, &result);
    classifier_as_.setSucceeded(result);
    ROS_INFO("REPLIED");
  } 

  void Classifier::AddItemActionCallback(const recycle_msgs::AddItemGoalConstPtr &goal)  {
    //Create result message
    recycle_msgs::AddItemResult result;
    ROS_INFO("ADDING ITEM");
    
    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");
    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(* msg, *cloud);

    Cropper cropper;
    PointCloudC::Ptr cropped = cropper.Crop(cloud);
    Downsampler downsampler;
    PointCloudC::Ptr downsampled = downsampler.Downsample(cropped);

    ros::NodeHandle nh;

    recycle::Segmenter segmenter;
    segmenter.AddItem(goal->category, downsampled, &result);

    ROS_INFO("REPLYING TO CLIENT");
    add_item_as_.setSucceeded(result);
    ROS_INFO("REPLIED");
  } 
}
