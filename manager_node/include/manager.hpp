#ifndef MANAGER_HPP_
#define MANAGER_HPP_
#include <mutex>
#include <future>
#include <unordered_map>
#include <octomap_msgs/conversions.h>
#include "camera_models/CameraFactory.h"
#include "camera_models/PinholeCamera.h"
#include "MyOcTree.h"
#include "keyframe.hpp"

class Manager
{
public:
    Manager(double resolution, uint frame_maintained = 0);
    inline double getResolution() {return _resolution;}
    inline octomap::MyOcTree* getOctree() {return _octree;}
    inline uint getMaintained() {return _frame_maintained;}
    inline uint getFrameCount() {return _frame_count;}
    inline void swapOctree() {
        octomap::MyOcTree* temp = _octree;
        // _octree->clear();
        // delete _octree;
        _octree = _loop_octree;
        // Clear _loop_octree in another thread
        _loop_octree = temp;
    };
    void setCamModel(string config_file);
    void initCoord(int row, int col, int step_size, int boundary);
    void addNewFrame(double time_stamp, Matrix3d& pose_R, Vector3d& pose_t,
                     const cv::Mat& depth_img, bool insert_flag);
    void removeLastFrame();
    void updateFrame(uint frame_index, Matrix3d& pose_R, Vector3d& pose_t);

    void newLoopTree();
    void deleteLoopTree();
    // Call inner lock
    void lock() {m_manager.lock();}
    bool try_lock() {return m_manager.try_lock();}
    void unlock() {m_manager.unlock();}

private:
    camodocal::CameraPtr _cam_model;
    octomap::MyOcTree* _octree;
    octomap::MyOcTree* _loop_octree;
    Matrix3d _Rci;
    Vector3d _tci;

    double _resolution;
    uint _frame_maintained;
    uint _frame_count;
    // Lock for loop
    mutex m_manager;
    // thread for delete loop tree
    std::future<void> _clear_loop_tree;
    unordered_map<uint, KeyFrame*> _keyframe_map;
    // Store fixed position in image plane
    vector<Vector2d> _fix_2d_coord;
    // Store fixed normalized points in camera coordinate
    vector<Vector3d> _fix_3d_coord;

};







#endif
