#include "recycle/feature_extraction.h"

#include <algorithm> // std::min and std::max

#include "recycle/object.h"
#include "recycle_msgs/ObjectFeatures.h"

namespace recycle {

  void ExtractColorFeatures(const Object& object,
                          recycle_msgs::ObjectFeatures* features) {
    //Determines weight given to color vs size in evaluating nearest neighbor
    // double weight;
    // ros::param::param("color_weight", weight, 1.0);

    std::vector<double> color_features;
    color_features.resize(3375); //15 buckets rather than 5, 15^3 = 3375
    for (size_t i = 0; i < object.cloud->size(); ++i) {
      const pcl::PointXYZRGB& pt = object.cloud->at(i);
      // Reduce the color space to just 15 values (255 / 17) per channel.
      int r = std::min(pt.r / 17, 14);
      int g = std::min(pt.g / 17, 14);
      int b = std::min(pt.b / 17, 14);
      int index = r * 225 + g * 15 + b;
      color_features[index] += 1;
    }

    // Normalize to get a distribution.
    for (size_t i = 0; i < color_features.size(); ++i) {
      color_features[i] /= object.cloud->size();
    }

    features->values.insert(features->values.end(), color_features.begin(),
                            color_features.end());
  } //ExtractColorFeatures

  void ExtractFeatures(const Object& object,
                       recycle_msgs::ObjectFeatures* features) {
    ExtractSizeFeatures(object, features);
    ExtractColorFeatures(object, features);
  } //ExtractFeatures

  void ExtractSizeFeatures(const Object& object,
                         recycle_msgs::ObjectFeatures* features) {
    //Determines weight given to size vs color in evaluating nearest neighbor
    // double weight;
    // ros::param::param("size_weight", weight, 2.0);
    
    // x is always the smallest dimension, y is always the second smallest and
    // z is always the largest. This accounts for any orientation of the item.
    // The bounding box dimensions will be generated separately and will not
    // be sorted in this way so the bot can accurately pick up items.
    double x_dim = object.dimensions.x;
    double y_dim = object.dimensions.y;
    double z_dim = object.dimensions.z;
    bool x_less_y = x_dim < y_dim;
    bool y_less_z = y_dim < z_dim;
    bool x_less_z = x_dim < z_dim;
    if (!x_less_y && y_less_z && x_less_z) {
      double tmp = x_dim;
      x_dim = y_dim;
      y_dim = tmp;
    } else if (x_less_y && !y_less_z && x_less_z) {
      double tmp = y_dim;
      y_dim = z_dim;
      z_dim = tmp;
    } else if (!x_less_y && y_less_z && !x_less_z) {
      double tmp = x_dim;
      x_dim = y_dim;
      y_dim = z_dim;
      z_dim = tmp;
    } else if (x_less_y && !y_less_z && !x_less_z) {
      double tmp = x_dim;
      x_dim = z_dim;
      z_dim = y_dim;
      y_dim = tmp;
    } else if (!x_less_y && !y_less_z && !x_less_z) {
      double tmp = x_dim;
      x_dim = z_dim;
      z_dim = tmp;
    }
    // double x_dim = 
    features->names.push_back("box_dim_x");
    features->values.push_back(x_dim);
    features->names.push_back("box_dim_y");
    features->values.push_back(y_dim);
    features->names.push_back("box_dim_z");
    features->values.push_back(z_dim);
  }//ExtractSizeFeatures
}  // namespace recycle
