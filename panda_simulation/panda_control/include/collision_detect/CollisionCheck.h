//
// Created by jieming on 11.12.20.
//

#ifndef NIHAO_COLLISIONCHECK_H
#define NIHAO_COLLISIONCHECK_H
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <urdf_parser/urdf_parser.h>

#include <urdf/model.h>

#include <geometric_shapes/mesh_operations.h>

#include <fcl/fcl.h>
#include "collision_detect/fcl_utility.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

using std::cout;
using std::endl;


class CollisionCheck {
    typedef double Real;
    typedef uint32_t UInt;
    typedef int32_t Int;

    struct Link {
        KDL::Frame transform;
        KDL::Frame *transformParent;
        std::string name;

        KDL::Frame transformToParent;
        const KDL::Joint *joint;
        const KDL::Segment* segment;

        Real *jointValue;
        std::shared_ptr<fcl::CollisionObjectd> collisionObject; //boost
        std::string mesh;
        bool isInCollision;

        Link():transformParent(nullptr), joint(nullptr), jointValue(nullptr), collisionObject(nullptr), segment(nullptr), isInCollision(false){}
    };

public:

    CollisionCheck();
    ~CollisionCheck();

    void publishCollisionModell();
    void publishInRange(const std::vector<Eigen::Vector3d>& near_points, std::vector<double>& sizes);
    void showBoxes();

    bool isFirstReceiv();

    void setOcTree(const octomap::OcTree* octree);

    void setDisabledLinkMapCollisions(const std::vector<std::string> &linksDisabled);

    bool isInCollision(const std::vector<Real> &joint_positions, const bool checkSelfCollision, const bool checkMapCollision);

    void getCollisions(const std::vector<Real> &joint_positions, std::vector<std::pair<std::string, std::string> > &selfCollisions,
                       std::vector<std::string> &mapCollisions);

    void getCollisions();

    void getCollisionsInRange(const double& range);

    void test_distance_spheresphere();

    // ******************** collision check server ********************

    bool callJointStateCB = true;

    bool checkCollision(const std::vector<Real> &jointPositions1);


private:

    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);

    bool initialize();

    bool initializeFCL();

    void findSelfCollisionPairs();

    bool parseSRDF(const std::string &filePath, std::vector<std::pair<std::string, std::string> > &disabledPairs);

    void getLinksFromString(const std::string &line, std::string &link1, std::string &link2) const;


    bool initializeKDL();

    void createLinksKDL();

    void expandTreeKDL(const KDL::SegmentMap::const_iterator& segment, UInt indexParent);


    // ******************** COLLISION CHECKING ********************
    void updateTransforms(const std::vector<Real> &jointPositions1);

    bool checkSelfCollision();

    void getSelfCollisions(std::vector<std::pair<std::string, std::string> > &selfCollisions);

    bool checkMapCollision();

    void getMapCollisions(std::vector<std::string> &mapCollisions);

    bool initialized;
    ros::NodeHandle nh;
    ros::Publisher publisherCollisionModell, traj_pub_, publisherNearObs;
    ros::Subscriber sensorSub;
    KDL::Tree kdlTree;
    std::vector<Real> jointPositions;
    std::vector<Link> links;

    std::vector<std::pair<Link*, Link*> > selfCollisionPairs;
    std::map<std::string, std::pair<Link*, bool> > octomapCollisionLinks;
    bool checkAllOcotmapLinkCollisions;

    boost::shared_ptr<fcl::CollisionObjectd> octomapCollisionObject;
    boost::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager1; //no use
    boost::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager2; //no use
    bool isFirstReceive;

    octomap::OcTree octTree_;
    std::shared_ptr<fcl::OcTree<double>> octreePtr_;
    std::vector<fcl::CollisionObjectd*> map_boxes;
    std::vector<double> box_sizes;

//    fcl::Sphered * box2 = new fcl::Sphered(0.2);
    //fcl::Boxd * box2 = new fcl::Boxd(0.2, 0.2, 0.2);  box->collisionGeometry().get()
};


#endif //NIHAO_COLLISIONCHECK_H
