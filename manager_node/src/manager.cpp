#include "manager.hpp"

Manager::Manager(double resolution, uint frame_maintained)
{
    temp_count = 0;
    _frame_count = 0;
    _resolution = resolution;
    _frame_maintained = frame_maintained;
    _octree = new octomap::MyOcTree(resolution);
    _octree->set_clamping_thres_max(500.0);
    _loop_octree = NULL;
    // Init future with a simple job
    _clear_loop_tree = std::async(std::launch::async, [](){
        cout << "Manager start working ..." << endl;
    });
    _clear_loop_tree.wait();
    // camera to imu
    _Rci << 0.0008748, -0.0047406,  0.9999884,
           -0.9999681, -0.0079460,  0.0008371,
            0.0079419, -0.9999572, -0.0047474;
    _tci << 1.30441e-05, 0.0149225, 0.000316835;
}

void Manager::setCamModel(string config_file)
{
    _cam_model = camodocal::CameraFactory::instance()->
                 generateCameraFromYamlFile(config_file.c_str());
}

void Manager::initCoord(int row, int col, int step_size, int boundary)
{
    for (int x = boundary; x < col - boundary; x += step_size)
    {
        for (int y = boundary; y < row - boundary; y += step_size)
        {
            Vector2d a(x, y);
            Vector3d b;
            _cam_model->liftProjective(a, b);
            _fix_2d_coord.push_back(a);
            _fix_3d_coord.push_back(b);
        }
    }
}

void Manager::addNewFrame(double time_stamp, Matrix3d& pose_R, Vector3d& pose_t,
                          const cv::Mat& depth_img, bool insert_flag)
{
    KeyFrame *keyframe;
    // Saving depth_img is not necessary in KeyFrame
    keyframe = new KeyFrame(time_stamp, _frame_count, pose_R, pose_t, depth_img);
    // Insert point to keyframe and octomap
    for (uint i = 0; i < _fix_2d_coord.size(); ++i)
    {
        // Retrieve depth from depth image and construct 3d world points
        double depth_val = (double)depth_img.at<unsigned short>(_fix_2d_coord[i](1),
                                                                _fix_2d_coord[i](0));
        if (depth_val == 0.0) continue;
        depth_val /= 1000;
        if (depth_val < 0.3 || depth_val > 4) continue;
        Vector3d point_3d_cam = _fix_3d_coord[i] * depth_val;
        // 3d points in camera coordinate with depth
        keyframe->insertPoint(point_3d_cam);
        // Add this flag to keep a good mapping performance
        if (insert_flag)
        {
            // Frome camera coordinate to world coordinate
            Vector3d point_3d_world = pose_R * (_Rci * point_3d_cam + _tci) + pose_t;
            //if (point_3d_world(2) > 0.3 && point_3d_world(2) < 0.6)
            _octree->insertRay(octomap::point3d(pose_t(0), pose_t(1), pose_t(2)),
                           octomap::point3d(point_3d_world(0),
                                            point_3d_world(1),
                                            point_3d_world(2)));

            // _octree->updateNode(octomap::point3d(point_3d_world(0),
            //                                      point_3d_world(1),
            //                                      point_3d_world(2)), true);
        }
    }

    _keyframe_map.insert(make_pair(_frame_count, keyframe));
    // If local mode, remove last keyframe
    // Remove after insertion is better
    if (_frame_maintained && _frame_count >= _frame_maintained)
        removeLastFrame();

    ++_frame_count;
}

void Manager::removeLastFrame()
{
    uint remove_id = _frame_count - _frame_maintained;
    cout<<"_frame_count:"<<_frame_count<<" !!!!  remove id: "<<remove_id<<endl;
    auto frame = _keyframe_map.find(remove_id);
    if (frame == _keyframe_map.end())
        return;

    cout<<"Given id found."<<endl;
    TicToc remove_time;
    // remove every point corressponds to this frame
    const vector<Vector3d> points_3d_cam = frame->second->getCamerePoints();
    auto R = frame->second->getR();
    auto t = frame->second->getT();
    for (uint i = 0; i < points_3d_cam.size(); ++i)
    {
        auto pt = R * (_Rci * points_3d_cam[i] + _tci) + t;
        _octree->tryDeleteNode(pt(0), pt(1), pt(2));
    }
    _keyframe_map.erase(frame);
    cout<<"remove takes: "<<remove_time.toc()<<endl;
}

void Manager::updateFrame(uint frame_index, Matrix3d& pose_R, Vector3d& pose_t)
{
    auto frame = _keyframe_map.find(frame_index);
    // Couldn't get value with given key
    if (frame == _keyframe_map.end())
        return;
    frame->second->updatePose(pose_R, pose_t);
    // Get 3d points in camera coordinate and reproject to world coordinate
    const vector<Vector3d> points_3d_cam = frame->second->getCamerePoints();
    temp_count += points_3d_cam.size();
    for (uint i = 0; i < points_3d_cam.size(); ++i)
    {
        Vector3d point_3d_world = pose_R * (_Rci * points_3d_cam[i] + _tci) + pose_t;
        _loop_octree->insertRay(octomap::point3d(pose_t(0), pose_t(1), pose_t(2)),
                                octomap::point3d(point_3d_world(0),
                                                 point_3d_world(1),
                                                 point_3d_world(2)));
        // _loop_octree->updateNode(octomap::point3d(point_3d_world(0),
        //                                           point_3d_world(1),
        //                                           point_3d_world(2)), true);
    }
}

void Manager::newLoopTree()
{
    _clear_loop_tree.wait();
    _loop_octree = new octomap::MyOcTree(_resolution);
}

void Manager::deleteLoopTree()
{
    _clear_loop_tree = std::async(std::launch::async, [&](){
        _loop_octree->clear();
        delete _loop_octree;
        _loop_octree = NULL;
    });
}
