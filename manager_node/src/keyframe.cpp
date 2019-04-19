#include "keyframe.hpp"

KeyFrame::KeyFrame(double time_stamp, uint frame_index, Matrix3d& pose_R,
                   Vector3d& pose_t, const cv::Mat& depth_img)
{
    // Init all the stuff
    _time_stamp = time_stamp;
    _frame_index = frame_index;
    _pose_R = pose_R;
    _pose_t = pose_t;
    _depth_img = depth_img;
}


void KeyFrame::updatePose(Matrix3d& pose_R, Vector3d& pose_t)
{
    // First update Odometry and update 3D points latter
    _pose_R = pose_R;
    _pose_t = pose_t;
}

void KeyFrame::insertPoint(Vector3d& point_3d_cam)
{
    _points_3d_cam.push_back(point_3d_cam);
}
