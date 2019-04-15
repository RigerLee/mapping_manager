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
    // camera to imu
    _Rci << 0.0008748, -0.0047406,  0.9999884,
           -0.9999681, -0.0079460,  0.0008371,
            0.0079419, -0.9999572, -0.0047474;
    _tci << 1.30441e-05, 0.0149225, 0.000316835;
}

void Manager::addNewFrame(double time_stamp, Matrix3d &pose_R, Vector3d &pose_t,
                          const cv::Mat &depth_img, bool insert_flag)
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
        if (depth_val < 0.3 || depth_val > 4) continue;
        Vector3d point_3d_cam = fix_3d_coord[i] * depth_val;
        // 3d points in camera coordinate with depth
        keyframe->insertPoint(point_3d_cam);
        // Add this flag to keep a good mapping performance
        if (insert_flag)
        {
            // Frome camera coordinate to world coordinate
            Vector3d point_3d_world = pose_R * (_Rci * point_3d_cam + _tci) + pose_t;
            _octree->insertRay(octomap::point3d(pose_t(0), pose_t(1), pose_t(2)),
                               octomap::point3d(point_3d_world(0),
                                                point_3d_world(1),
                                                point_3d_world(2)));

            // _octree->updateNode(octomap::point3d(point_3d_world(0),
            //                                      point_3d_world(1),
            //                                      point_3d_world(2)), true);
        }
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
        Vector3d point_3d_world = pose_R * (_Rci * points_3d_cam[i] + _tci) + pose_t;
        temp_octree->insertRay(octomap::point3d(pose_t(0), pose_t(1), pose_t(2)),
                               octomap::point3d(point_3d_world(0),
                                                point_3d_world(1),
                                                point_3d_world(2)));
    }
}
