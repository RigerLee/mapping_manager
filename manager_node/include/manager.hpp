#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "keyframe.hpp"

class Manager
{
public:
    Manager(double resolution);
    inline double getResolution() {return _resolution;}
    inline octomap::OcTree* getOctree() {return _octree;}
    inline void setOctree(octomap::OcTree* octree) {
      _octree->clear();
      delete _octree;
      _octree = octree;
      octree = NULL;
    };
    void addNewFrame(double time_stamp, Matrix3d &pose_R,
                     Vector3d &pose_t, const cv::Mat &depth_img);
    void updateFrame(int frame_index, Matrix3d &pose_R, Vector3d &pose_t,
                     octomap::OcTree* temp_octree);
private:
    int _frame_count;
    double _resolution;
    vector<KeyFrame*> _keyframe_vector;
    octomap::OcTree* _octree;
};







#endif
