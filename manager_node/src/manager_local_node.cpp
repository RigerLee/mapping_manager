#include <thread>
#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include "manager.hpp"

using namespace std;
using namespace Eigen;

typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image> syncPolicy;

// Message buffer
queue<nav_msgs::Odometry::ConstPtr> local_pose_buf;
queue<sensor_msgs::PointCloudConstPtr> loop_pose_buf;
queue<sensor_msgs::ImageConstPtr> depth_buf;
mutex m_local_buf, m_loop_buf;

ros::Publisher pub_octomap;

bool visualize_octomap = true;

void local_callback(const nav_msgs::Odometry::ConstPtr& pose_msg,
                   const sensor_msgs::ImageConstPtr& depth_msg)
{
    m_local_buf.lock();
    local_pose_buf.push(pose_msg);
    depth_buf.push(depth_msg);
    m_local_buf.unlock();
}

void loop_callback(const sensor_msgs::PointCloudConstPtr& pcl_msg)
{
    m_loop_buf.lock();
    loop_pose_buf.push(pcl_msg);
    m_loop_buf.unlock();
}

void local_run(Manager& manager)
{
    vector<pair<bool, nav_msgs::Odometry::ConstPtr>> pose_msg_vect;
    vector<sensor_msgs::ImageConstPtr> depth_msg_vect;
    while (true)
    {
        // Get message from buffer
        m_local_buf.lock();
        while (!local_pose_buf.empty())
        {
            pose_msg_vect.push_back(make_pair(true, local_pose_buf.front()));
            depth_msg_vect.push_back(depth_buf.front());
            local_pose_buf.pop();
            depth_buf.pop();
        }
        m_local_buf.unlock();

        // mutex m_manager is for local and loop communication
        if (manager.try_lock())
        {

            for (uint i = 0; i < pose_msg_vect.size(); ++i)
            {
                auto pose_msg = pose_msg_vect[i].second;
                auto depth_msg = depth_msg_vect[i];
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
                TicToc add_frame_time;
                // try_lock()  Lock manager
                manager.addNewFrame(pose_msg->header.stamp.toSec(),
                                    pose_R, pose_t, depth_ptr->image,
                                    pose_msg_vect[i].first);
                if (visualize_octomap)
                {
                    // publish
                    octomap_msgs::Octomap map;
                    map.header = pose_msg->header;
                    if (octomap_msgs::fullMapToMsg(*(manager.getOctree()), map))
                        pub_octomap.publish(map);
                    else
                        ROS_ERROR("Error serializing OctoMap");
                }
                ROS_INFO("Add new local frame done, time cost %f ms", add_frame_time.toc());
            }
            manager.unlock();
            pose_msg_vect.clear();
            depth_msg_vect.clear();
        }
        else
        {
            for (auto& pose_msg_pair : pose_msg_vect)
                // In case that pose immediately after loop may not accurate
                // Won't actually add 3D points to octomap but create keyframe
                pose_msg_pair.first = false;
        }
        chrono::milliseconds dura(5);
        this_thread::sleep_for(dura);
    }
}

void loop_run(Manager& manager)
{
    while (true)
    {
        sensor_msgs::PointCloudConstPtr pcl_msg = NULL;

        // Get message from buffer
        m_loop_buf.lock();
        if (!loop_pose_buf.empty())
        {
            pcl_msg = loop_pose_buf.front();
            loop_pose_buf.pop();
        }
        m_loop_buf.unlock();


        if (pcl_msg != NULL)
        {
            // Lock manager, prepare for update
            manager.lock();
            TicToc loop_time;
            ROS_WARN("Loop start");
            manager.newLoopTree();
            // TODO: bug here, out of range!!!
            cout<<pcl_msg->points.size()<< "  should no larger than  "<< manager.getFrameCount()<<endl;
            uint start = manager.getFrameCount() >= manager.getMaintained() ?
                         manager.getFrameCount() - manager.getMaintained() : 0;
            for (uint i = start; i < pcl_msg->points.size(); ++i)
            {
                Vector3d pose_t(pcl_msg->points[i].x,
                                pcl_msg->points[i].y,
                                pcl_msg->points[i].z);
                Quaterniond pose_q;
                pose_q.w() = pcl_msg->channels[0].values[i];
                pose_q.x() = pcl_msg->channels[1].values[i];
                pose_q.y() = pcl_msg->channels[2].values[i];
                pose_q.z() = pcl_msg->channels[3].values[i];
                Matrix3d pose_R = pose_q.toRotationMatrix();

                manager.updateFrame(i, pose_R, pose_t);
            }
            manager.swapOctree();
            manager.deleteLoopTree();
            ROS_WARN("Loop done, time cost %f ms", loop_time.toc());
            // Unlock manager, update done
            manager.unlock();
            chrono::milliseconds dura(1000);
            this_thread::sleep_for(dura);
        }

        chrono::milliseconds dura(5);
        this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping_manager");
    ros::NodeHandle n("~");
    // Obtain config file, read parameters and release
    string config_file;
    n.getParam("config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if (!fsSettings.isOpened())
        cerr << "ERROR: Can not open config file!" << endl;
    // Set resolution for octomap
    double resolution = fsSettings["resolution"];
    // Set frame_maintained for local map
    int frame_maintained = fsSettings["frame_maintained"];
    // Set topics need to listen
    string pose_topic, depth_topic, local_topic, loop_topic;
    fsSettings["pose_topic"] >> pose_topic;
    fsSettings["depth_topic"] >> depth_topic;
    fsSettings["loop_topic"] >> loop_topic;
    // Some settings about density of pointcloud
    int row, col, step_size, boundary, max_depth, min_depth;
    row = fsSettings["image_height"];
    col = fsSettings["image_width"];
    step_size = fsSettings["step_size"];
    boundary = fsSettings["boundary"];
    max_depth = fsSettings["max_depth"];
    min_depth = fsSettings["min_depth"];
    fsSettings.release();

    Manager manager(resolution, (uint)frame_maintained);
    // TODO: manager.setParameter();

    // Init camera model
    manager.setCamModel(config_file);
    // Init fixed 2D points in image plane
    manager.initCoord(row, col, step_size, boundary);
    // Init synchronizer for camera pose and depth map
    message_filters::Subscriber<nav_msgs::Odometry> sub_pose(n, pose_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, depth_topic, 1);
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_pose, sub_depth);
    sync.registerCallback(boost::bind(&local_callback, _1, _2));
    // Init other subscriber
    ros::Subscriber sub_loop = n.subscribe(loop_topic, 1000, loop_callback);
    // Init advertiser
    pub_octomap = n.advertise<octomap_msgs::Octomap>("octomap_full", 1, true);

    cout<<"Build octomap with resolution: "<<resolution<<"m "<<endl;
    cout<<"Down sample depth map with step size: "<<step_size<<endl;
    // Main thread
    thread local_process = thread(local_run, ref(manager));
    thread loop_process = thread(loop_run, ref(manager));

    ros::spin();

    return 0;
}
