#include "MyOcTree.h"


namespace octomap {

	MyOcTree::MyOcTree(double in_resolution)
		: OccupancyOcTreeBase<OcTreeNode>(in_resolution) {
		ocTreeMemberInit.ensureLinking();
	};

  MyOcTree::MyOcTree(std::string _filename)
    : OccupancyOcTreeBase<OcTreeNode> (0.1)  { // resolution will be set according to tree file
    readBinary(_filename);
  }

	bool MyOcTree::decreaseNodeLogOdds(OcTreeNode* occupancyNode,
																		 const float& update) const {
 		occupancyNode->addValue(update);
		if (occupancyNode->getLogOdds() < 1) {
			occupancyNode->setLogOdds(this->clamping_thres_min);
			return true;
		}
		return false;
	}

	void MyOcTree::tryDeleteNode(double x, double y, double z) {
		OcTreeNode* my_node = this->search(x, y, z);
		if (my_node == NULL) return;
		if (decreaseNodeLogOdds(my_node, 0-this->prob_hit_log))
			this->deleteNode(x, y, z);
	}


  MyOcTree::StaticMemberInitializer MyOcTree::ocTreeMemberInit;




} // namespace
