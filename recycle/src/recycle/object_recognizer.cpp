#include "recycle/object_recognizer.h"

#include <limits.h>
#include <math.h>
#include <string>
#include <vector>
#include <cmath>

#include "boost/filesystem.hpp"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "recycle/feature_extraction.h"
#include "recycle_msgs/ObjectFeatures.h"
#include "ros/ros.h"
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using boost::filesystem::directory_iterator;
using recycle_msgs::ObjectFeatures;

namespace recycle {

namespace {
  double EuclideanDistance(const std::vector<double>& v1,
                           const std::vector<double>& v2) {
      // TODO: implement
    double distance = 0.0;
    double size_weight;
    ros::param::param("size_weight", size_weight, 2.0);
    double color_weight;
    ros::param::param("color_weight", color_weight, 1.0);
    int n = v1.size();
    for (int i = 0; i < n; i++) {
      double dist = std::pow(v1.at(i) - v2.at(i), 2);
      if (i < 3) {
        dist *= size_weight;
      } else {
        dist *= color_weight;
      }
      distance += dist;
    }
    return std::sqrt(distance);
  }
} // anonymous namespace

// log_id INTEGER PRIMARY KEY AUTOINCREMENT,
//   predicted_category VARCHAR(255),
//   actual_category VARCHAR(255),
//   image_file_path VARCHAR(255),
//   pointcloud_file_path VARCHAR(255),
//   feature_file_path VARCHAR(255)


int SQLCallback(void *data, int argc, char **argv, char **azColName) {
  // printf("%s = %s\n", azColName[0], argv[0] ? argv[0] : "NULL");

  // ROS_INFO_STREAM("INSIDE CALLBACK ADDRESS: " << dataset_point);
  std::string type = argv[0];
  std::vector<recycle_msgs::ObjectFeatures>* dataset_point = static_cast<std::vector<recycle_msgs::ObjectFeatures>*>(data);
  // ROS_INFO_STREAM("Loading " << argv[1]);
  rosbag::Bag bag;
  bag.open(argv[1], rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back("object_features");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it) {
    ObjectFeatures::Ptr fp = it->instantiate<ObjectFeatures>();
    if (fp != NULL) {
      // ROS_INFO_STREAM("ITEM HAS THE DATA " << fp->names[0]);
      // ROS_INFO_STREAM("Original type is " << fp->classification);
      fp->classification = type;
      // ROS_INFO_STREAM("New type is " << fp->classification);
      dataset_point->push_back(*fp);
    }
  }
  // ROS_INFO_STREAM("SIZE NOW " << dataset_point->size());
  bag.close();
  return 0;
}

int ObjectRecognizer::LoadData(const std::string& database_path) {
  // ROS_INFO_STREAM("OUTSIDE ADDRESS: " << &dataset_);

  sqlite3 *db;
  char *zErrMsg = 0;
  int rc = sqlite3_open(database_path.c_str(), &db);
  if(rc) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    sqlite3_close(db);
    return -1;
  } else {
    fprintf(stderr, "Opened database successfully\n");
  }

  std::string statement = "SELECT actual_category, feature_file_path FROM classification_log WHERE actual_category is not NULL";
  
  rc = sqlite3_exec(db, statement.c_str(), SQLCallback, static_cast<void*>(&dataset_), &zErrMsg);

  if(rc != SQLITE_OK ) {
    fprintf(stderr, "SQL error: %s\n", zErrMsg);
    sqlite3_free(zErrMsg);
   } else {
      fprintf(stdout, "Operation done successfully\n");
   }

   sqlite3_close(db);
   ROS_INFO_STREAM("DATASET SIZE AFTER " << dataset_.size());
   return 0;
}

ObjectRecognizer::ObjectRecognizer() {}

void ObjectRecognizer::Recognize(Object& object, std::string* name,
                                 double* confidence) {
  // TODO: extract features from the object
  recycle_msgs::ObjectFeatures features;
  // features.classification = "unknown";
  recycle::ExtractFeatures(object, &features);

  std::vector<double> feature_values(features.values);

  double min_distance = std::numeric_limits<double>::max();
  double second_min_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < dataset_.size(); ++i) {
    // TODO: compare the features of the input object to the features of the current dataset object.
    std::vector<double> labelled_values(dataset_.at(i).values);
    double distance = recycle::EuclideanDistance(feature_values, labelled_values);
    if (distance < min_distance) {
      second_min_distance = min_distance;
      min_distance = distance;
      *name = dataset_[i].classification;
      object.name = dataset_[i].classification;
    } else if (distance < second_min_distance) {
      second_min_distance = distance;
    }
  }

  // Confidence is based on the distance to the two nearest results.
  *confidence = 1 - min_distance / (min_distance + second_min_distance);
  object.confidence = *confidence;
}

}  // namespace recycle
