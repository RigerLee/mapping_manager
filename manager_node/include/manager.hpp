#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "keyframe.hpp"

class Manager
{
public:
    Manager(double _resolution);
    void addNewFrame(double _time_stamp, int _frame_index, Matrix3d &_pose_R,
                     Vector3d &_pose_t, cv::Mat &_depth_img);
private:
    octomap::OcTree* octree;
    vector<KeyFrame *> keyframe_vector;
};







#endif
