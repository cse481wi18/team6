#ifndef _RECYCLE_FEATURE_EXTRACTION_H_
#define _RECYCLE_FEATURE_EXTRACTION_H_

#include "ros/ros.h"
#include "recycle/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace recycle {
	void ExtractColorFeatures(const Object& object,
                          perception_msgs::ObjectFeatures* features);

    void ExtractSizeFeatures(const Object& object,
                                     perception_msgs::ObjectFeatures* features);

	void ExtractFeatures(const Object& object,
	                     perception_msgs::ObjectFeatures* features);
}  // namespace recycle

#endif  // _RECYCLE_FEATURE_EXTRACTION_H_
