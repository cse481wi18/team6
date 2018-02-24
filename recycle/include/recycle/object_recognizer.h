#ifndef _RECYCLE_OBJECT_RECOGNIZER_H_
#define _RECYCLE_OBJECT_RECOGNIZER_H_

#include <string>
#include <vector>

#include "pcl/kdtree/kdtree_flann.h"

#include "recycle/object.h"
#include "perception_msgs/ObjectFeatures.h"

namespace recycle {
void LoadData(const std::string& data_dir,
              std::vector<perception_msgs::ObjectFeatures>* dataset);

class ObjectRecognizer {
 public:
  explicit ObjectRecognizer(
      const std::vector<perception_msgs::ObjectFeatures>& dataset);
  void Recognize(const Object& object, std::string* name, double* confidence);

 private:
  std::vector<perception_msgs::ObjectFeatures> dataset_;
};
}  // namespace recycle

#endif  // _RECYCLE_OBJECT_RECOGNIZER_H_