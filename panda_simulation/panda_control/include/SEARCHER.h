//
// Created by jieming on 02.12.20.
//

#ifndef PANDA_CONTROL_SEARCHER_H
#define PANDA_CONTROL_SEARCHER_H
#include "common.h"

#include <fcl/fcl.h>
#include <octomap_msgs/Octomap.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
 #include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/pdst/PDST.h>

#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/sst/SST.h>

#include <ompl/geometric/planners/prm/PRM.h>

#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <msg_pkg/Collisioncheck.h>

#include "collision_detect/CollisionCheck.h"


namespace ob = ompl::base;
namespace og = ompl::geometric;

class SEARCHER {
private:
    // construct the state space we are planning in
    ob::StateSpacePtr space_;
    // construct an instance of space information from this state space
    ob::SpaceInformationPtr si_;
    // create a problem instance
    ob::ProblemDefinitionPtr pdef_;
    // goal state
    double prev_goal_[3];

    og::PathGeometric *path_smooth_ = NULL;

    bool replan_flag_ = false;

    ros::NodeHandle n_;
    ros::Publisher traj_pub_;
    ros::Publisher vis_pub_;
    ros::Publisher trajPub_;
    ros::ServiceClient client_ ;

    CollisionCheck collision_check_;
    bool isStateValid(const ob::State *state);

public:
    // Constructor
    SEARCHER();

    // Destructor
    ~SEARCHER();

    void setStart(const std::array<double, 7> &);

    void setGoal(const std::array<double, 7> &);

    void replan();

    std::optional< vector<array<double,7>>> plan();

    vector<array<double,7>> getStates(const ompl::geometric::PathGeometric *path);
    //-------------------debug use--------------------
    void  getStatesShow( const ompl::geometric::PathGeometric* path);

    void pubTrajectory(const std::array<double, 7> &);
};
#endif //PANDA_CONTROL_SEARCHER_H
