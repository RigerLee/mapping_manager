#include <mutex>
#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "manager.hpp"
#include "camera_models/CameraFactory.h"
#include "camera_models/PinholeCamera.h"

using namespace std;
using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> syncPolicy;

vector<Vector2d> fix_2d_coord;
vector<Vector3d> fix_3d_coord;
// Camera model of depth camera
camodocal::CameraPtr cam_model;
// Message buffer
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<sensor_msgs::ImageConstPtr> depth_buf;
mutex m_buf;

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg,
                   const sensor_msgs::ImageConstPtr &depth_msg)
{
    m_buf.lock();
    pose_buf.push(pose_msg);
    depth_buf.push(depth_msg);
    m_buf.unlock();
}

void local_callback(const nav_msgs::Odometry::ConstPtr &pose_msg){}
void loop_callback(const nav_msgs::Odometry::ConstPtr &pose_msg){}

void initCoord(int row, int col, int step_size, int u_boundary, int b_boundary,
               int l_boundary, int r_boundary, int max_depth, int min_depth)
{
    for (int x = l_boundary; x < col - r_boundary; x += step_size)
    {
        for (int y = u_boundary; y < row - b_boundary; y += step_size)
        {
            Vector2d a(x, y);
            Vector3d b;
            cam_model->liftProjective(a, b);
            fix_2d_coord.push_back(a);
            fix_3d_coord.push_back(b);
        }
    }
}

void run(Manager &manager)
{
    while (true)
    {
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;
        sensor_msgs::ImageConstPtr depth_msg = NULL;

        // Get message from buffer
        m_buf.lock();
        if(!pose_buf.empty())
        {
            pose_msg = pose_buf.front();
            depth_msg = depth_buf.front();
        }
        m_buf.unlock();

        if (pose_msg != NULL && depth_msg != NULL)
        {
            // Recover R and t from odometry message
            Vector3d pose_t(pose_msg->pose.pose.position.x,
                                   pose_msg->pose.pose.position.y,
                                   pose_msg->pose.pose.position.z);
            Quaterniond pose_q;
            pose_q.w() = pose_msg->pose.pose.orientation.w;
            pose_q.x() = pose_msg->pose.pose.orientation.x;
            pose_q.y() = pose_msg->pose.pose.orientation.y;
            pose_q.z() = pose_msg->pose.pose.orientation.z;
            Matrix3d pose_R = pose_q.toRotationMatrix();

            // Recover depth map from depth message
            cv_bridge::CvImageConstPtr depth_ptr;
            {
                sensor_msgs::Image msg;
                msg.header = depth_msg->header;
                msg.height = depth_msg->height;
                msg.width = depth_msg->width;
                msg.is_bigendian = depth_msg->is_bigendian;
                msg.step = depth_msg->step;
                msg.data = depth_msg->data;
                msg.encoding = sensor_msgs::image_encodings::MONO16;
                depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
            }
            cv::Mat depth_img = depth_ptr->image;
            // TODO: need to get frame_index
            int frame_index = 1;
            manager.addNewFrame(pose_msg->header.stamp.toSec(), frame_index,
                                pose_R, pose_t, depth_img);
        }

        chrono::milliseconds dura(5);
        this_thread::sleep_for(dura);
    }


}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_manager");
    ros::NodeHandle n("~");
    ROS_INFO("Started running, waiting for poses and depth images.");
    // Obtain config file, read parameters and release
    string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
        cerr << "ERROR: Can not open config file!" << endl;
    // Set resolution for octomap
    double resolution = fsSettings["resolution"];
    // Set topics need to listen
    string pose_topic, depth_topic, local_topic, loop_topic;
    fsSettings["pose_topic"] >> pose_topic;
    fsSettings["depth_topic"] >> depth_topic;
    fsSettings["local_topic"] >> local_topic;
    fsSettings["loop_topic"] >> loop_topic;
    // Some settings about density of pointcloud
    int row, col, step_size, u_boundary, d_boundary, l_boundary, r_boundary, max_depth, min_depth;
    row = fsSettings["image_height"];
    col = fsSettings["image_width"];
    step_size = fsSettings["step_size"];
    u_boundary = fsSettings["u_boundary"];
    d_boundary = fsSettings["d_boundary"];
    l_boundary = fsSettings["l_boundary"];
    r_boundary = fsSettings["r_boundary"];
    max_depth = fsSettings["max_depth"];
    min_depth = fsSettings["min_depth"];
    fsSettings.release();

    // Init camera model
    cam_model = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(config_file.c_str());
    initCoord(row, col, step_size, u_boundary, d_boundary,
              l_boundary, r_boundary, max_depth, min_depth);
    // Init synchronizer for camera pose and depth map
    message_filters::Subscriber<nav_msgs::Odometry> sub_pose(n, pose_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, depth_topic, 1);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_pose, sub_depth);
    sync.registerCallback(boost::bind(&pose_callback, _1, _2));
    // Init other subscriber
    ros::Subscriber sub_local = n.subscribe(local_topic, 1000, local_callback);
    ros::Subscriber sub_loop = n.subscribe(loop_topic, 1000, loop_callback);

    // Main thread
    Manager manager(resolution);
    thread manager_process = thread(run, ref(manager));

    ros::spin();

    return 0;
}
