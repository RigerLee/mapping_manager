#include "keyframe.hpp"

KeyFrame::KeyFrame(double _time_stamp, int _frame_index, Matrix3d &_pose_R, Vector3d &_pose_t)
{
    time_stamp = _time_stamp;
    frame_index = _frame_index;
    pose_R = _pose_R;
    pose_t = _pose_t;
}


void KeyFrame::updatePose(Matrix3d &_pose_R, Vector3d &_pose_t)
{
    pose_R = _pose_R;
    pose_t = _pose_t;
}

void KeyFrame::insertPoint(int point_id, Vector3d &point3d)
{
    ListIter iter = depth_list.insert(depth_list.begin(), make_pair(point_id, point3d));
    iter_vector.push_back(iter);
}

// const list<pair<int, Vector3d>> &getDepthList()
// {
//     return &depth_list;
// }
// const vector<ListIter> &getIterVector();
// {
//     return &iter_vector;
// }
