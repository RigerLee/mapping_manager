#ifndef KEYFRAME_HPP_
#define KEYFRAME_HPP_

#include <list>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
typedef list<pair<int, Vector3d>>::iterator ListIter;

class KeyFrame
{
public:
    KeyFrame(double _time_stamp, int _frame_index, Matrix3d &_pose_R, Vector3d &_pose_t);

    void updatePose(Matrix3d &_pose_R, Vector3d &_pose_t);
    void insertPoint(int point_id, Vector3d &point3d);

    //const list<pair<int, Vector3d>> &getDepthList();
    //const vector<ListIter> &getIterVector();

private:
    double time_stamp;
    int frame_index;
    Matrix3d pose_R;
    Vector3d pose_t;
    // depth_list: Doubly linked list, store <id, depth>
    // iter_vector: Iterator vector, store iterator of elements in depth_list
    // This can speedup loop closure (where hard to be real time)
    list<pair<int, Vector3d>> depth_list;
    vector<ListIter> iter_vector;
};















#endif
