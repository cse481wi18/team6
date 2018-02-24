#ifndef _RECYCLE_BOX_FITTER_H_
#define _RECYCLE_BOX_FITTER_H_

#include "geometry_msgs/Pose.h"
#include "pcl/ModelCoefficients.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "shape_msgs/SolidPrimitive.h"

namespace recycle {
bool FitBox(const pcl::PointCloud<pcl::PointXYZRGB>& input,
            const pcl::ModelCoefficients::Ptr model,
            pcl::PointCloud<pcl::PointXYZRGB>& output,
            shape_msgs::SolidPrimitive& shape, geometry_msgs::Pose& pose);
}  // namespace recycle

#endif // _RECYCLE_BOX_FITTER_H_