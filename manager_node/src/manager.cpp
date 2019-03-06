#include "manager.hpp"

extern vector<Vector2d> fix_2d_coord;
extern vector<Vector3d> fix_3d_coord;

Manager::Manager(double _resolution)
{
    octree = new octomap::OcTree(_resolution);
}

void Manager::addNewFrame(double _time_stamp, int _frame_index, Matrix3d &_pose_R,
                          Vector3d &_pose_t, cv::Mat &_depth_img)
{
    KeyFrame *keyframe = new KeyFrame(_time_stamp, _frame_index, _pose_R, _pose_t);

    for (int i = 0, point_id = 0; i < fix_2d_coord.size(); ++i)
    {
        double depth_val = (double)_depth_img.at<unsigned short>(fix_2d_coord[i](1),
                                                                 fix_2d_coord[i](0));
        if (depth_val == 0.0) continue;
        depth_val /= 1000;
        Vector3d point3d = fix_3d_coord[i] * depth_val;
        // TODO: strategy for insertion
        if (!true)
            continue;
        // Only important points will be inserted
        octree->updateNode(octomap::point3d(point3d(0), point3d(1), point3d(2)), true);
        keyframe->insertPoint(point_id, point3d);
        ++point_id;
    }

    keyframe_vector.push_back(keyframe);
}
