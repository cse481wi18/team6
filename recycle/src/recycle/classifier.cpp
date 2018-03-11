#include "recycle/classifier.h"

namespace recycle {
  Classifier::Classifier(std::string database_path, 
                        std::string classify_name,
                        std::string add_item_name,
                        std::string logger_name) :
    classifier_as_(nh_, classify_name, boost::bind(&Classifier::ClassifierActionCallback, this, _1), false),
    add_item_as_(nh_, add_item_name, boost::bind(&Classifier::AddItemActionCallback, this, _1), false),
    logging_ac_(logger_name, true), 
    database_path_(database_path),
    classifier_action_name_(classify_name),
    add_item_action_name_(add_item_name),
    logger_name_(logger_name) {
      crop_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
      above_table_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("above_table_cloud", 1, true);
      classifier_as_.start();
      add_item_as_.start();
      logging_ac_.waitForServer();
      ROS_INFO("INIT COMPLETE");
    }

  void Classifier::ClassifierActionCallback(const recycle_msgs::ClassifyGoalConstPtr &goal)  {
    recycle_msgs::ClassifyResult result;

    ROS_INFO("CLASSIFYING");
    
    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("cloud_in");
    
    tf::TransformListener tf_listener;                                                    
    tf_listener.waitForTransform("base_link", msg->header.frame_id,                     
                                 ros::Time(0), ros::Duration(5.0));                       
    tf::StampedTransform transform;                                                       
    try {                                                                                 
      tf_listener.lookupTransform("base_link", msg->header.frame_id,                    
                                  ros::Time(0), transform);                               
    } catch (tf::LookupException& e) {                                                    
      std::cerr << e.what() << std::endl;                                                 
      // return 1;                                                                           
    } catch (tf::ExtrapolationException& e) {                                             
      std::cerr << e.what() << std::endl;                                                 
      // return 1;                                                                           
    }                                                                                     
                                                                                            
    sensor_msgs::PointCloud2 cloud_out;                                                   
    pcl_ros::transformPointCloud("base_link", transform, *msg, cloud_out);

    PointCloudC::Ptr cloud(new PointCloudC());
    pcl::fromROSMsg(cloud_out, *cloud);

    Cropper cropper(crop_pub_);
    PointCloudC::Ptr cropped = cropper.Crop(cloud);
    Downsampler downsampler;
    PointCloudC::Ptr downsampled = downsampler.Downsample(cropped);

    recycle::ObjectRecognizer recognizer;
    recognizer.LoadData(database_path_);
    ROS_INFO("Segmenting");
    recycle::Segmenter segmenter(above_table_pub_, recognizer);
    segmenter.SegmentAndClassify(downsampled, &result, &logging_ac_);
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

    Cropper cropper(crop_pub_);
    PointCloudC::Ptr cropped = cropper.Crop(cloud);
    Downsampler downsampler;
    PointCloudC::Ptr downsampled = downsampler.Downsample(cropped);
    ROS_INFO("Segmenting");
    recycle::Segmenter segmenter(above_table_pub_);
    segmenter.AddItem(goal->category, downsampled, &result);

    add_item_as_.setSucceeded(result);
    ROS_INFO("REPLIED");
  } 
}
