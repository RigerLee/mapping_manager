#ifndef MANAGER_HPP_
#define MANAGER_HPP_
#include <mutex>
#include <unordered_map>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include "camera_models/CameraFactory.h"
#include "camera_models/PinholeCamera.h"
#include "keyframe.hpp"

class Manager
{
public:
    Manager(double resolution, uint frame_maintained = 0);
    inline double getResolution() {return _resolution;}
    inline octomap::OcTree* getOctree() {return _octree;}
    inline uint getFrameCount() {return _frame_count;}
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
    void removeLastFrame();
    void updateFrame(uint frame_index, Matrix3d& pose_R, Vector3d& pose_t,
                     octomap::OcTree* temp_octree);
    // Call inner lock
    void lock() {m_manager.lock();};
    bool try_lock() {return m_manager.try_lock();};
    void unlock() {m_manager.unlock();};
private:
    camodocal::CameraPtr _cam_model;
    octomap::OcTree* _octree;
    Matrix3d _Rci;
    Vector3d _tci;

    double _resolution;
    uint _frame_maintained;
    uint _frame_count;
    // Lock for loop
    mutex m_manager;
    unordered_map<uint, KeyFrame*> _keyframe_map;
    // Store fixed position in image plane
    vector<Vector2d> _fix_2d_coord;
    // Store fixed normalized points in camera coordinate
    vector<Vector3d> _fix_3d_coord;

};







#endif
