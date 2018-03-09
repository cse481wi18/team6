#ifndef _RECYCLE_OBJECT_RECOGNIZER_H_
#define _RECYCLE_OBJECT_RECOGNIZER_H_

#include <string>
#include <set>
#include <vector>

#include "pcl/kdtree/kdtree_flann.h"

#include "recycle/object.h"
#include "recycle_msgs/ObjectFeatures.h"

#include <sqlite3.h>

namespace recycle {


	int SQLCallback(void *data, int argc, char **argv, char **azColName);
	class ObjectRecognizer {
	 	public:

	  	explicit ObjectRecognizer();
	  	void Recognize(Object& object, std::string* name, double* confidence);
		int LoadData(const std::string& database_path);

	 	private:
		std::vector<recycle_msgs::ObjectFeatures> dataset_;
	};
}  // namespace recycle

#endif  // _RECYCLE_OBJECT_RECOGNIZER_H_