#include "manager.hpp"
// Store fixed position in image plane
extern vector<Vector2d> fix_2d_coord;
// Store fixed normalized points in camera coordinate
extern vector<Vector3d> fix_3d_coord;

Manager::Manager(double resolution)
{
    _frame_count = 0;
    _resolution = resolution;
    _octree = new octomap::OcTree(resolution);
}

void Manager::addNewFrame(double time_stamp, Matrix3d &pose_R,
                          Vector3d &pose_t, const cv::Mat &depth_img)
{
    KeyFrame *keyframe;
    // Saving depth_img is not necessary in KeyFrame
    keyframe = new KeyFrame(time_stamp, _frame_count, pose_R, pose_t, depth_img);
    // Insert point to keyframe and octomap
    for (uint i = 0; i < fix_2d_coord.size(); ++i)
    {
        // Retrieve depth from depth image and construct 3d world points
        double depth_val = (double)depth_img.at<unsigned short>(fix_2d_coord[i](1),
                                                                fix_2d_coord[i](0));
        if (depth_val == 0.0) continue;
        depth_val /= 1000;
        Vector3d point_3d_cam = fix_3d_coord[i] * depth_val;
        // 3d points in camera coordinate
        keyframe->insertPoint(point_3d_cam);
        // Frome camera coordinate to world coordinate
        Vector3d point_3d_world = pose_R * point_3d_cam + pose_t;

        _octree->updateNode(octomap::point3d(point_3d_world(0),
                                            point_3d_world(1),
                                            point_3d_world(2)), true);

    }

    _keyframe_vector.push_back(keyframe);
    ++_frame_count;
}

void Manager::updateFrame(int frame_index, Matrix3d &pose_R, Vector3d &pose_t,
                          octomap::OcTree* temp_octree)
{
    _keyframe_vector[frame_index]->updatePose(pose_R, pose_t);
    // Get 3d points in camera coordinate and reproject to world coordinate
    const vector<Vector3d> points_3d_cam = _keyframe_vector[frame_index]->getCamerePoints();
    for (uint i = 0; i < points_3d_cam.size(); ++i)
    {
        Vector3d point_3d_world = pose_R * points_3d_cam[i] + pose_t;
        temp_octree->updateNode(octomap::point3d(point_3d_world(0),
                                                 point_3d_world(1),
                                                 point_3d_world(2)), true);
    }
}
