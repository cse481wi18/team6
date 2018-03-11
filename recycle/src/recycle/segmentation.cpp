#include "recycle/segmentation.h"
#include "recycle/box_fitter.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "recycle/object.h"

#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/extract_clusters.h"
#include "visualization_msgs/Marker.h"
#include "simple_grasping/shape_extraction.h"
#include "shape_msgs/SolidPrimitive.h"
#include "geometry_msgs/PoseStamped.h"

#include <math.h>
#include <sstream>
#include "recycle/object_recognizer.h"

namespace recycle {

void Segmenter::SegmentSurface(PointCloudC::Ptr cloud,
          pcl::PointIndices::Ptr indices,
          pcl::ModelCoefficients::Ptr coeff) {
  pcl::PointIndices indices_internal;
  pcl::SACSegmentation<PointC> seg;
  seg.setOptimizeCoefficients(true);
  // Search for a plane perpendicular to some axis (specified below).
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  // Set the distance to the plane for a point to be an inlier.
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);

  // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
  Eigen::Vector3f axis;
  axis << 0, 0, 1;
  seg.setAxis(axis);
  double table_slant_tolerance;
  ros::param::param("table_slant_tolerance", table_slant_tolerance, 10.0);
  seg.setEpsAngle(pcl::deg2rad(table_slant_tolerance));

  // coeff contains the coefficients of the plane:
  // ax + by + cz + d = 0
  seg.segment(indices_internal, *coeff);

  // get rid of things on top of the table
  double distance_above_plane;
  ros::param::param("distance_above_plane", distance_above_plane, 0.005);

  // Build custom indices that ignores points above the plane.
  for (size_t i = 0; i < cloud->size(); ++i) {
    const PointC& pt = cloud->points[i];
    float val = coeff->values[0] * pt.x + coeff->values[1] * pt.y +
                coeff->values[2] * pt.z + coeff->values[3];
    if (val <= distance_above_plane) {
      indices->indices.push_back(i);
    }
  }

  if (indices->indices.size() == 0) {
    ROS_ERROR("Unable to find surface.");
    return;
  }
}

void Segmenter::SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                           pcl::PointIndices::Ptr surface_indices,
                           std::vector<pcl::PointIndices>* object_indices,
                           bool classify) {
  pcl::ExtractIndices<PointC> extract;
  pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
  extract.setInputCloud(cloud);
  extract.setIndices(surface_indices);
  extract.setNegative(true);
  extract.filter(above_surface_indices->indices);

  ROS_INFO("There are %ld points above the table", above_surface_indices->indices.size());

  double cluster_tolerance;
  int min_cluster_size, max_cluster_size;
  int min_cluster_size_add, max_cluster_size_add;
  ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
  ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
  ros::param::param("ec_max_cluster_size", max_cluster_size, 200);
  ros::param::param("ec_min_cluster_size_add", min_cluster_size_add, 10);
  ros::param::param("ec_max_cluster_size_add", max_cluster_size_add, 200);

  pcl::EuclideanClusterExtraction<PointC> euclid;
  euclid.setInputCloud(cloud);
  euclid.setIndices(above_surface_indices);
  euclid.setClusterTolerance(cluster_tolerance);
  if (classify) {
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
  } else {
    euclid.setMinClusterSize(min_cluster_size_add);
    euclid.setMaxClusterSize(max_cluster_size_add);
  }
  euclid.extract(*object_indices);

  // Find the size of the smallest and the largest object,
  // where size = number of points in the cluster
  size_t min_size = std::numeric_limits<size_t>::max();
  size_t max_size = std::numeric_limits<size_t>::min();
  for (size_t i = 0; i < object_indices->size(); ++i) {
    // TODO: implement this
    size_t cluster_size = object_indices->at(i).indices.size();
    if (cluster_size < min_size) {
      min_size = cluster_size;
    }
    if (cluster_size > max_size) {
      max_size = cluster_size;
    }
  }

  ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
           object_indices->size(), min_size, max_size);
}

void Segmenter::SegmentTabletopScene(PointCloudC::Ptr cloud,
                          std::vector<Object>* objects,
                          std::vector<Object>* obstacles,
                          PointCloudC::Ptr above_surface_cloud,
                          bool classify) {
  // Same as callback, but with visualization code removed.
  pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  Segmenter::SegmentSurface(cloud, table_inliers, coeff);

  PointCloudC::Ptr table_cloud(new PointCloudC());

  // Extract subset of original_cloud into table_cloud:
  pcl::ExtractIndices<PointC> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(table_inliers);
  extract.filter(*table_cloud);

  // sensor_msgs::PointCloud2 msg_out;
  // pcl::toROSMsg(*table_cloud, msg_out);
  // table_cloud_pub_.publish(msg_out); 

  //Use simple_grasping instead of our own function
  PointCloudC::Ptr extract_out(new PointCloudC());
  
  // Bounding box for the table
  shape_msgs::SolidPrimitive table_shape;
  geometry_msgs::Pose table_pose;
  FitBox(*table_cloud, coeff, *extract_out, table_shape, table_pose);
      
  recycle::Object *table = new recycle::Object;
  ROS_INFO("%s", table_cloud->header.frame_id.c_str());
  table->cloud = table_cloud;
  table->pose = table_pose;
  table->dimensions.x = table_shape.dimensions[0];
  table->dimensions.y = table_shape.dimensions[1];
  table->dimensions.z = table_shape.dimensions[2];
  // ROS_INFO_STREAM("TABLE DIMENSIONS " << table_shape);
  // ROS_INFO_STREAM("TABLE POSE " << table_pose);

  obstacles->push_back(*table);

  // segmenting surface objects
  std::vector<pcl::PointIndices> object_indices;
  Segmenter::SegmentSurfaceObjects(cloud, table_inliers, &object_indices, classify);
  
  extract.setNegative(true);
  extract.filter(*above_surface_cloud);
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*above_surface_cloud, msg_out);
  above_table_pub_.publish(msg_out); 
  
  // bounding box for objects
  for (size_t i = 0; i < object_indices.size(); ++i) {
    // Reify indices into a point cloud of the object.
    pcl::PointIndices::Ptr indices(new pcl::PointIndices);
    *indices = object_indices[i];
    PointCloudC::Ptr object_cloud(new PointCloudC());
    // TODO: fill in object_cloud using indices
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*object_cloud);

    shape_msgs::SolidPrimitive object_shape;
    geometry_msgs::Pose object_pose;
    FitBox(*object_cloud, coeff, *extract_out, object_shape, object_pose);
        
    recycle::Object *new_object = new recycle::Object;
    new_object->cloud = object_cloud;
    new_object->pose = object_pose;
    new_object->dimensions.x = object_shape.dimensions[0];
    new_object->dimensions.y = object_shape.dimensions[1];
    new_object->dimensions.z = object_shape.dimensions[2];

    objects->push_back(*new_object);
  }
}

Segmenter::Segmenter(ros::Publisher above_table_pub, const ObjectRecognizer& recognizer) : above_table_pub_(above_table_pub),
recognizer_(recognizer) {
  CONFIDENCE_THRESHOLD = 0.5;
}

Segmenter::Segmenter(ros::Publisher above_table_pub) : above_table_pub_(above_table_pub) {  
  CONFIDENCE_THRESHOLD = 0.5;
}

void Segmenter::AddItem(std::string category,
                        PointCloudC::Ptr cloud_unfiltered, 
                        recycle_msgs::AddItemResult* result) {
  PointCloudC::Ptr cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);  

  std::vector<Object> objects;
  std::vector<Object> obstacles;
  PointCloudC::Ptr above_surface_cloud(new PointCloudC);
  Segmenter::SegmentTabletopScene(cloud, &objects, &obstacles, above_surface_cloud, false);
  if (objects.size() != 1) {
    ROS_INFO_STREAM("Expected to find 1 object. Found " << objects.size());
  }

  int item_index = 0;
  double largest_vol = -1;
  for (int i = 0; i < objects.size(); i++) {
    Object& current_object = objects[i];
    double current_vol = current_object.dimensions.x * current_object.dimensions.y * current_object.dimensions.z;
    if (current_vol > largest_vol) {
      item_index = i;
      largest_vol = current_vol;
    }
  }

  Object& object = objects[item_index];
  object.name = category;
  recycle_msgs::ObjectFeatures features;
  recycle::ExtractFeatures(object, &features);
  saver_.Log(object, &features, &(result->to_log), false);
}

void Segmenter::SegmentAndClassify(PointCloudC::Ptr cloud_unfiltered, 
                                   recycle_msgs::ClassifyResult* result,
                                   actionlib::SimpleActionClient<recycle_msgs::DbLogAction>* ac) {

  PointCloudC::Ptr cloud(new PointCloudC());
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);  

  std::vector<Object> objects;
  std::vector<Object> obstacles;
  PointCloudC::Ptr above_surface_cloud(new PointCloudC);
  Segmenter::SegmentTabletopScene(cloud, &objects, &obstacles, above_surface_cloud, true);

  result->num_obstacles = obstacles.size();
  for (size_t i = 0; i < obstacles.size(); ++i) {
    const Object& obstacle = obstacles[i];  
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "base_link";
    poseStamped.pose = obstacle.pose;
    result->obstacle_poses.push_back(poseStamped);
    result->obstacle_dimensions.push_back(obstacle.dimensions);
  }

  result->num_objects = objects.size();
  ROS_INFO_STREAM("Found " << objects.size() << " objects");
  for (size_t i = 0; i < objects.size(); ++i) {
    Object& object = objects[i];  
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "base_link";
    poseStamped.pose = object.pose;
    result->object_poses.push_back(poseStamped);
    result->object_dimensions.push_back(object.dimensions);

    // Recognize the object.
    std::string name;
    double confidence;
    recognizer_.Recognize(object, &name, &confidence);
    confidence = round(1000 * confidence) / 1000;

    if (confidence > CONFIDENCE_THRESHOLD) {
      result->classifications.push_back(name);
    } else {
      result->classifications.push_back("landfill");
    }
    result->confidence.push_back(confidence);

    // Save and reply to the logger
    recycle_msgs::ObjectFeatures features;
    recycle::ExtractFeatures(object, &features);
    recycle_msgs::LogItem item;
    saver_.Log(object, &features, &item, true);
    recycle_msgs::DbLogGoal goal;
    goal.log_item = item;
    ac->sendGoal(goal);
  }
}
}  // namespace recycle