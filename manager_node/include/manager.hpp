#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "camera_models/CameraFactory.h"
#include "camera_models/PinholeCamera.h"
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
    void setCamModel(string config_file);
    void initCoord(int row, int col, int step_size, int boundary);
    void addNewFrame(double time_stamp, Matrix3d& pose_R, Vector3d& pose_t,
                     const cv::Mat& depth_img, bool insert_flag);
    void updateFrame(int frame_index, Matrix3d& pose_R, Vector3d& pose_t,
                     octomap::OcTree* temp_octree);
    int _frame_count;
private:
    camodocal::CameraPtr _cam_model;
    octomap::OcTree* _octree;
    Matrix3d _Rci;
    Vector3d _tci;

    double _resolution;
    vector<KeyFrame*> _keyframe_vector;
    // Store fixed position in image plane
    vector<Vector2d> _fix_2d_coord;
    // Store fixed normalized points in camera coordinate
    vector<Vector3d> _fix_3d_coord;

};







#endif
