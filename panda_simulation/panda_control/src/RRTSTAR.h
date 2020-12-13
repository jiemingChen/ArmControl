//
// Created by jieming on 02.12.20.
//

#ifndef PANDA_CONTROL_RRTSTAR_H
#define PANDA_CONTROL_RRTSTAR_H
#include "common.h"

/*
#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/shape/geometric_shapes.h>
*/
#include <fcl/fcl.h>
#include <octomap_msgs/Octomap.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>



namespace ob = ompl::base;
namespace og = ompl::geometric;


class Searcher {
private:
    // construct the state space we are planning in
    ob::StateSpacePtr space_;
    // construct an instance of space information from this state space
    ob::SpaceInformationPtr si_;
    // create a problem instance
    ob::ProblemDefinitionPtr pdef_;
    // goal state
    double prev_goal_[3];

    og::PathGeometric* path_smooth_ = NULL;

    bool replan_flag_ = false;

    std::shared_ptr<fcl::CollisionGeometryd> robot_;

    std::shared_ptr<fcl::CollisionGeometryd> tree_obj_;

    ros::NodeHandle n_;
    ros::Publisher traj_pub_;
    ros::Publisher vis_pub_;

    bool isStateValid(const ob::State *state){
        // cast the abstract state type to the type we expect
        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        fcl::CollisionObjectd treeObj((tree_obj_));
        fcl::CollisionObjectd robotObject(robot_);

        // check validity of state defined by pos & rot
        fcl::Vector3d translation(pos->values[0],pos->values[1],pos->values[2]);
        fcl::Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);
        robotObject.setTransform(rotation, translation);
        fcl::CollisionRequestd requestType(1,false,1,false);
        fcl::CollisionResultd collisionResult;
        fcl::collide(&robotObject, &treeObj, requestType, collisionResult);

        return(!collisionResult.isCollision());
    }

public:
    // Constructor
    Searcher(){
        traj_pub_ = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
        vis_pub_ = n_.advertise<visualization_msgs::Marker>( "rrt_marker", 10 );

        std::vector<fcl::Vector3f> vertices;
        std::vector<fcl::Triangle> triangles;
        typedef fcl::BVHModel<fcl::OBBRSSf> Model;
        std::shared_ptr<Model> geom = std::make_shared<Model>();
        geom->beginModel();
        geom->addSubModel(vertices, triangles);
        geom->endModel();

        robot_ = std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Boxd(0.6, 0.5, 0.5));
        fcl::OcTreed* tree = new fcl::OcTreed(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
        tree_obj_ = std::shared_ptr<fcl::CollisionGeometryd>(tree);

        space_ = ob::StateSpacePtr(new ob::SE3StateSpace());
        // create a start state
        ob::ScopedState<ob::SE3StateSpace> start(space_);
        // create a goal state
        ob::ScopedState<ob::SE3StateSpace> goal(space_);

        // set the bounds for the R^3 part of SE(3)
        ob::RealVectorBounds bounds(3);
        bounds.setLow(0,-10); bounds.setHigh(0,10);
        bounds.setLow(1,-10); bounds.setHigh(1,10);
        bounds.setLow(2,0);   bounds.setHigh(2,20);

        space_->as<ob::SE3StateSpace>()->setBounds(bounds);
        // construct an instance of  space information from this state space
        si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));

        start->setXYZ(0,0,0);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // start.random();

        goal->setXYZ(0,0,0);
        prev_goal_[0] = 0; prev_goal_[1] = 0; prev_goal_[2] = 0;
        goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        // goal.random();

        // set state validity checking for this space
        si_->setStateValidityChecker(std::bind(&Searcher::isStateValid, this, std::placeholders::_1 ));

        // create a problem instance
        pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));

        // set the start and goal states
        pdef_->setStartAndGoalStates(start, goal);

        std::cout << "Initialized: " << std::endl;
    }
    // Destructor
    ~Searcher(){};

    void setStart(double x, double y, double z){
        ob::ScopedState<ob::SE3StateSpace> start(space_);
        start->setXYZ(x,y,z);
        start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
        pdef_->clearStartStates();
        pdef_->addStartState(start);
    }

    void setGoal(double x, double y, double z){
        if(prev_goal_[0] != x || prev_goal_[1] != y || prev_goal_[2] != z){
            ob::ScopedState<ob::SE3StateSpace> goal(space_);
            goal->setXYZ(x,y,z);
            prev_goal_[0] = x;
            prev_goal_[1] = y;
            prev_goal_[2] = z;
            goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();
            pdef_->clearGoal();
            pdef_->setGoalState(goal);
            std::cout << "Goal point set to: " << x << " " << y << " " << z << std::endl;
        }
    }

    void updateMap(std::shared_ptr<fcl::CollisionGeometryd> map){
        tree_obj_ = map;
    }
#if 0
    void replan(void)
    {
        if(path_smooth != NULL && set_start)
        {
            std::cout << "Total Points:" << path_smooth->getStateCount () << std::endl;
            if(path_smooth->getStateCount () <= 2)
                plan();
            else
            {
                for (std::size_t idx = 0; idx < path_smooth->getStateCount (); idx++)
                {
                    if(!replan_flag)
                        replan_flag = !isStateValid(path_smooth->getState(idx));
                    else
                        break;

                }
                if(replan_flag)
                    plan();
                else
                    std::cout << "Replanning not required" << std::endl;
            }
        }
    }
#endif
    void plan(){
        // create a planner for the defined space
        ob::PlannerPtr plan(new og::InformedRRTstar(si_));

        // set the problem we are trying to solve for the planner
        plan->setProblemDefinition(pdef_);

        // perform setup steps for the planner
        plan->setup();

        // print the settings for this space
        si_->printSettings(std::cout);

        // print the problem settings
        pdef_->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = plan->solve(1);
        if (solved){
            // get the goal representation from the problem definition (not the same as the goal state)
            // and inquire about the found path
            std::cout << "Found solution:" << std::endl;
            ob::PathPtr path = pdef_->getSolutionPath();
            og::PathGeometric* pth = pdef_->getSolutionPath()->as<og::PathGeometric>();
            pth->printAsMatrix(std::cout);
            // print the path to screen
            // path->print(std::cout);
            trajectory_msgs::MultiDOFJointTrajectory msg;
            trajectory_msgs::MultiDOFJointTrajectoryPoint point_msg;

            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "panda1/world";
            msg.joint_names.clear();
            msg.points.clear();
            msg.joint_names.push_back("Quadcopter");

            for (std::size_t path_idx = 0; path_idx < pth->getStateCount (); path_idx++){
                const ob::SE3StateSpace::StateType *se3state = pth->getState(path_idx)->as<ob::SE3StateSpace::StateType>();

                // extract the first component of the state and cast it to what we expect
                const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // extract the second component of the state and cast it to what we expect
                const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                point_msg.time_from_start.fromSec(ros::Time::now().toSec());
                point_msg.transforms.resize(1);

                point_msg.transforms[0].translation.x= pos->values[0];
                point_msg.transforms[0].translation.y = pos->values[1];
                point_msg.transforms[0].translation.z = pos->values[2];

                point_msg.transforms[0].rotation.x = rot->x;
                point_msg.transforms[0].rotation.y = rot->y;
                point_msg.transforms[0].rotation.z = rot->z;
                point_msg.transforms[0].rotation.w = rot->w;

                msg.points.push_back(point_msg);

            }
            traj_pub_.publish(msg);

#if 1
            //Path smoothing using bspline

            og::PathSimplifier* pathBSpline = new og::PathSimplifier(si_);
            path_smooth_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef_->getSolutionPath()));
            pathBSpline->smoothBSpline(*path_smooth_,3);
            // std::cout << "Smoothed Path" << std::endl;
            // path_smooth.print(std::cout);


            //Publish path as markers

            visualization_msgs::Marker marker;

            for (std::size_t idx = 0; idx < path_smooth_->getStateCount (); idx++){
                // cast the abstract state type to the type we expect
                const ob::SE3StateSpace::StateType *se3state = path_smooth_->getState(idx)->as<ob::SE3StateSpace::StateType>();

                // extract the first component of the state and cast it to what we expect
                const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

                // extract the second component of the state and cast it to what we expect
                const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

                marker.header.frame_id = "panda1/world";
                marker.header.stamp = ros::Time();
                marker.ns = "rrt_path";
                marker.id = idx;
                marker.type = visualization_msgs::Marker::CUBE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = pos->values[0];
                marker.pose.position.y = pos->values[1];
                marker.pose.position.z = pos->values[2];
                marker.pose.orientation.x = rot->x;
                marker.pose.orientation.y = rot->y;
                marker.pose.orientation.z = rot->z;
                marker.pose.orientation.w = rot->w;
                marker.scale.x = 0.15;
                marker.scale.y = 0.15;
                marker.scale.z = 0.15;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                vis_pub_.publish(marker);
                // ros::Duration(0.1).sleep();
             }
#endif
            // Clear memory
            pdef_->clearSolutionPaths();
            replan_flag_ = false;

        }
        else
            std::cout << "No solution found" << std::endl;
    }

};

//
//void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg, Searcher* planner_ptr){
//#if 1
//    //loading octree from binary
//     const std::string filename = "/home/jieming/catkin_ws/src/panda_simulation/panda_simulation/maps/test.bt";
//     octomap::OcTree temp_tree(0.05);
//     temp_tree.readBinary(filename);
//     fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(&temp_tree));
//#else
//     // convert octree to collision object
//    octomap::OcTree* tree_oct = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
//   fcl::OcTree* tree = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree_oct));
//#endif
//    // Update the octree used for collision checking
//    planner_ptr->updateMap(std::shared_ptr<fcl::CollisionGeometry>(tree));
//    planner_ptr->plan();
//}


class RRTSTAR {

};
#endif //PANDA_CONTROL_RRTSTAR_H
