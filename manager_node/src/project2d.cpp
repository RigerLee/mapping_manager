#include "project2d.hpp"
OccupancyMap::OccupancyMap(octomap::MyOcTree* octree) : _octree(octree){
}

void OccupancyMap::publish2Dmap(const std_msgs::Header& header, double zmin, double zmax){
  // alloc memory stuff
  handle_pre(header);
  if (zmin >= zmax) {
    zmax = std::numeric_limits<double>::max();
    zmin = -zmax;
  }

  // now, traverse all leafs in the tree:
  for (octomap::MyOcTree::iterator it = _octree->begin(16),
      end = _octree->end(); it != end; ++it)
  {

    if (_octree->isNodeOccupied(*it)){
      double z = it.getZ();
      if (z > zmin && z < zmax)
        update2DMap(it, true);
    } else{ // node not occupied => mark as free in 2D map
      double z = it.getZ();
      //if (z > zmin && z < zmax)
        update2DMap(it, false);
    }
  }
}

void OccupancyMap::handle_pre(const std_msgs::Header& header){
  // init projected 2D map:
  _gridmap.header.frame_id = header.frame_id;
  _gridmap.header.stamp = header.stamp;

  // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
  double minX, minY, minZ, maxX, maxY, maxZ;
  _octree->getMetricMin(minX, minY, minZ);
  _octree->getMetricMax(maxX, maxY, maxZ);

  octomap::point3d minPt(minX, minY, minZ);
  octomap::point3d maxPt(maxX, maxY, maxZ);
  octomap::OcTreeKey minKey = _octree->coordToKey(minPt, 16);
  octomap::OcTreeKey maxKey = _octree->coordToKey(maxPt, 16);

  // ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

  // add padding if requested (= new min/maxPts in x&y):
  // default: m_minSizeX(0.0), m_minSizeY(0.0)
  minX = std::min(minX, 0.0);
  maxX = std::max(maxX, 0.0);
  minY = std::min(minY, 0.0);
  maxY = std::max(maxY, 0.0);
  minPt = octomap::point3d(minX, minY, minZ);
  maxPt = octomap::point3d(maxX, maxY, maxZ);

  octomap::OcTreeKey paddedMaxKey;
  if (!_octree->coordToKeyChecked(minPt, 16, _paddedMinKey)){
    ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
    return;
  }
  if (!_octree->coordToKeyChecked(maxPt, 16, paddedMaxKey)){
    ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
    return;
  }

  ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", _paddedMinKey[0], _paddedMinKey[1], _paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
  assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

  _gridmap.info.width = paddedMaxKey[0] - _paddedMinKey[0] +1;
  _gridmap.info.height = paddedMaxKey[1] - _paddedMinKey[1] +1;

  int mapOriginX = minKey[0] - _paddedMinKey[0];
  int mapOriginY = minKey[1] - _paddedMinKey[1];
  assert(mapOriginX >= 0 && mapOriginY >= 0);

  // might not exactly be min / max of octree:
  octomap::point3d origin = _octree->keyToCoord(_paddedMinKey, 16);
  double gridRes = _octree->getNodeSize(16);
  _gridmap.info.resolution = gridRes;
  _gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
  _gridmap.info.origin.position.y = origin.y() - gridRes*0.5;


  ROS_DEBUG("Rebuilding complete 2D map");
  _gridmap.data.clear();
  _gridmap.data.resize(_gridmap.info.width * _gridmap.info.height, -1);
}


void OccupancyMap::update2DMap(const octomap::MyOcTree::iterator& it, bool occupied){

  // update 2D map (occupied always overrides):
  if (it.getDepth() == 16){
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
      // _gridmap.data[mapIdx(it.getKey())] = 100;
      _gridmap.data[idx] = 100;
    else if (_gridmap.data[idx] == -1){
      _gridmap.data[idx] = 0;
    }

  } else{
    int intSize = 1 << (16 - it.getDepth());
    octomap::OcTreeKey minKey=it.getIndexKey();
    for(int dx=0; dx < intSize; dx++){
      int i = minKey[0]+dx - _paddedMinKey[0];
      for(int dy=0; dy < intSize; dy++){
        unsigned idx = mapIdx(i, minKey[1]+dy - _paddedMinKey[1]);
        if (occupied)
          _gridmap.data[idx] = 100;
        else if (_gridmap.data[idx] == -1){
          _gridmap.data[idx] = 0;
        }
      }
    }
  }
}
