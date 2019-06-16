#pragma once

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include "MyOcTree.h"
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

class OccupancyMap {
public:
  OccupancyMap(octomap::MyOcTree* octree);
  ~OccupancyMap();

  void publish2Dmap(const std_msgs::Header& header);
  void handle_pre(const std_msgs::Header& header);
  void update2DMap(const octomap::MyOcTree::iterator& it, bool occupied);

  const nav_msgs::OccupancyGrid& getGridmap() {return _gridmap;}
  void setOctree(octomap::MyOcTree* octree) {_octree = octree;}

  inline unsigned mapIdx(int i, int j) const {
    return _gridmap.info.width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const {
    return mapIdx(key[0] - _paddedMinKey[0], key[1] - _paddedMinKey[1]);
  }

private:
  octomap::MyOcTree* _octree;
  nav_msgs::OccupancyGrid _gridmap;
  octomap::OcTreeKey _paddedMinKey;
  double _occupancyMinZ;
  double _occupancyMaxZ;
};