#ifndef KEYFRAME_HPP_
#define KEYFRAME_HPP_

#include <vector>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <tic_toc.hpp>

using namespace std;
using namespace Eigen;
// typedef list<pair<int, Vector3d>>::iterator ListIter;

class KeyFrame
{
public:
    KeyFrame(double time_stamp, uint frame_index, Matrix3d& pose_R,
             Vector3d& pose_t, const cv::Mat& depth_img);

    void updatePose(Matrix3d& pose_R, Vector3d& pose_t);
    void insertPoint(Vector3d& points_3d_cam);

    inline const vector<Vector3d>& getCamerePoints() {return _points_3d_cam;}

    //const list<pair<int, Vector3d>> &getDepthList();
    //const vector<ListIter> &getIterVector();

private:
    double _time_stamp;
    uint _frame_index;
    Matrix3d _pose_R;
    Vector3d _pose_t;
    cv::Mat _depth_img;
    vector<Vector3d> _points_3d_cam;

};


#endif
