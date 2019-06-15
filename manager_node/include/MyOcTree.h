#ifndef MY_OCTREE_H
#define MY_OCTREE_H


#include <octomap/OccupancyOcTreeBase.h>
#include <octomap/ScanGraph.h>
#include <octomap/OcTreeNode.h>

namespace octomap {

  /**
   * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
   * Basic functionality is implemented in OcTreeBase.
   *
   */
  class MyOcTree : public OccupancyOcTreeBase <OcTreeNode> {

  public:
    /// Default constructor, sets resolution of leafs
    MyOcTree(double resolution);

    /**
     * Reads an OcTree from a binary file
    * @param _filename
     *
     */
    MyOcTree(std::string _filename);

    virtual ~MyOcTree(){};

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    MyOcTree* create() const {return new MyOcTree(resolution); }

    std::string getTreeType() const {return "OcTree";}

    bool decreaseNodeLogOdds(OcTreeNode* occupancyNode,
  													 const float& update) const;

    void tryDeleteNode(double x, double y, double z);
    void set_clamping_thres_max(float x);

  protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
    public:
      StaticMemberInitializer() {
        MyOcTree* tree = new MyOcTree(0.1);
        tree->clearKeyRays();
        AbstractOcTree::registerTreeType(tree);
      }

	    /**
	     * Dummy function to ensure that MSVC does not drop the
	     * StaticMemberInitializer, causing this tree failing to register.
	     * Needs to be called from the constructor of this octree.
	     */
	    void ensureLinking() {};
    };

    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeMemberInit;
  };

} // end namespace

#endif
