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
#include "recycle/object_recognizer.h"
#include "recycle_msgs/ClassifyAction.h"

namespace recycle {
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

class Segmenter {
 public:
  Segmenter(const ros::Publisher& table_cloud_pub,
            const ObjectRecognizer& recognizer);
  
  // Does a complete tabletop segmentation pipeline.
  //
  // Args:
  //  cloud: The point cloud with the surface and the objects above it.
  //  objects: The output objects.
  void SegmentTabletopScene(PointCloudC::Ptr cloud,
                            std::vector<Object>* objects, 
                            std::vector<Object>* obstacles,
                            PointCloudC::Ptr above_surface_cloud);
  

  void SegmentAndClassify(PointCloudC::Ptr cloud_unfiltered, 
                          recycle_msgs::ClassifyResult* result);

 private:
  ros::Publisher table_cloud_pub_;
  ObjectRecognizer recognizer_;
  int CONFIDENCE_THRESHOLD;

  // Finds the largest horizontal surface in the given point cloud.
  // This is useful for adding a collision object to MoveIt.
  //
  // Args:
  //  cloud: The point cloud to extract a surface from.
  //  indices: The indices of points in the point cloud that correspond to the
  //    surface. Empty if no surface was found.
  void SegmentSurface(PointCloudC::Ptr cloud,
                      pcl::PointIndices::Ptr indices,
                      pcl::ModelCoefficients::Ptr coeff);


  void SegmentSurfaceObjects(PointCloudC::Ptr cloud,
                             pcl::PointIndices::Ptr surface_indices,
                             std::vector<pcl::PointIndices>* object_indices);
 };
}  // namespace recycle
