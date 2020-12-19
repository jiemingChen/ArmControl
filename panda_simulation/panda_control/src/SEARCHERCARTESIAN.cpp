//
// Created by jieming on 02.12.20.
//

#include "SEARCHERCARTESIAN.h"
// private
#if 0
bool SEARCHER::isStateValid(const ob::State *state){
//    return true;

    msg_pkg::Collisioncheck srv;
    vector<float> joint_pos(7);
    srv.request.start = true;

    for(size_t j=0; j<7; j++){
        joint_pos[j] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
     }
    srv.request.joint_pos = joint_pos;

     if (!client_.call(srv)){
         ROS_ERROR("Failed to call service collision_check_srv");
         return false;
     }
    return true;

    return !srv.response.collide;
}
#endif
ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si);

ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& si)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
    return obj;
}

bool SEARCHER_CARTESIAN::isStateValid(const ob::State *state){
//    return true;
    vector<double> joint_pos(7);

    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    Eigen::Affine3d transform;
    transform.translate(Eigen::Vector3d(pos->values[0], pos->values[1], pos->values[2]));
    Eigen::Quaterniond temp( rot->w,  rot->x,  rot->y,  rot->z);
    transform.linear()  = temp.matrix();

    robot_->setJoints(Eigen::Map<Eigen::Vector7d>(Initial_joints_.data()), Eigen::Vector7d::Zero());
    auto joints = robot_->iK(transform);
    if(joints[0] < -99)
        return false;

    for(auto i=0; i<7; i++){
        joint_pos[i] = joints[i];
    }
     return !collision_check_.checkCollision(joint_pos);
 }

//public
void SEARCHER_CARTESIAN::set_joints_for_IK(std::array<double, 7> initial_joints){
    Initial_joints_ = initial_joints;
}

SEARCHER_CARTESIAN::SEARCHER_CARTESIAN(Panda* robot){
    robot_ = robot;
    client_ = n_.serviceClient<msg_pkg::Collisioncheck>("collision_check_srv");
    traj_pub_ = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
    vis_pub_ = n_.advertise<visualization_msgs::Marker>( "rrt_marker", 10 );
    trajPub_ =  n_.advertise<trajectory_msgs::JointTrajectory>("/robot1/jointTrajectory", 2);

    collision_check_.callJointStateCB = false;

    // Create the state space for the manipulator
    space_ = ob::StateSpacePtr(new ob::SE3StateSpace());    // create a start state
    ob::ScopedState<ob::SE3StateSpace> start(space_);    // create a goal state
    ob::ScopedState<ob::SE3StateSpace> goal(space_);

    // Bound the joints of the manipulator between [-PI, PI]
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0,-2); bounds.setHigh(0,2);
    bounds.setLow(1,-2); bounds.setHigh(1,2);
    bounds.setLow(2,0); bounds.setHigh(2,2);
    space_->as<ob::SE3StateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
    si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));
    si_->setStateValidityChecker(std::bind(&SEARCHER_CARTESIAN::isStateValid, this, std::placeholders::_1));
    si_->setup();
//    si_->printSettings();
    // set state validity checking for this space
    start->setXYZ(0,0,0);
    start->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    goal->setXYZ(0,0,0);
    goal->as<ob::SO3StateSpace::StateType>(1)->setIdentity();

    // create a problem instance
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    // set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);

    pdef_->setOptimizationObjective( getPathLengthObjWithCostToGo(si_));

    std::cout << "initialized " << std::endl;
}
// Destructor
SEARCHER_CARTESIAN::~SEARCHER_CARTESIAN(){
 //    delete path_smooth_;
};

void SEARCHER_CARTESIAN::setStart(const Eigen::Affine3d& start_trans){
    ob::ScopedState<ob::SE3StateSpace> start(space_);
    start->setXYZ(start_trans.translation()(0), start_trans.translation()(1), start_trans.translation()(2));
    Eigen::Quaterniond temp(start_trans.linear());

    start->as<ob::SO3StateSpace::StateType>(1)->w =  temp.w();
    start->as<ob::SO3StateSpace::StateType>(1)->x =  temp.x();
    start->as<ob::SO3StateSpace::StateType>(1)->y =  temp.y();
    start->as<ob::SO3StateSpace::StateType>(1)->z =  temp.z();

    pdef_->clearStartStates();
    pdef_->addStartState(start);

}

void SEARCHER_CARTESIAN::setGoal(const Eigen::Affine3d& goal_trans){
    ob::ScopedState<ob::SE3StateSpace> goal(space_);

    goal->setXYZ(goal_trans.translation()(0), goal_trans.translation()(1), goal_trans.translation()(2));
    Eigen::Quaterniond temp(goal_trans.linear());

    goal->as<ob::SO3StateSpace::StateType>(1)->w =  temp.w();
    goal->as<ob::SO3StateSpace::StateType>(1)->x =  temp.x();
    goal->as<ob::SO3StateSpace::StateType>(1)->y =  temp.y();
    goal->as<ob::SO3StateSpace::StateType>(1)->z =  temp.z();


    pdef_->clearGoal();
    pdef_->setGoalState(goal);
}

void SEARCHER_CARTESIAN::replan(){
//    if(path_smooth_ != NULL && set_start){
    if(path_smooth_ != NULL){
        std::cout << "Total Points:" << path_smooth_->getStateCount () << std::endl;
        if(path_smooth_->getStateCount () <= 2)
            plan();
        else
        {
            for (std::size_t idx = 0; idx < path_smooth_->getStateCount (); idx++)
            {
                if(!replan_flag_)
                    replan_flag_ = !isStateValid(path_smooth_->getState(idx));
                else
                    break;

            }
            if(replan_flag_)
                plan();
            else
                std::cout << "Replanning not required" << std::endl;
        }
    }
}

std::optional< vector<array<double,3>>> SEARCHER_CARTESIAN::plan(){
//    pdef_->setOptimizationObjective(getPathLengthObjective(si_));
    //    auto optimizingPlanner(std::make_shared<og::InformedRRTstar>(si_));
//    optimizingPlanner->setTreePruning(true);

//    auto optimizingPlanner(std::make_shared<og::RRT>(si_));  //no good
//    auto optimizingPlanner(std::make_shared<og::PRM>(si_));  //no good
//    auto optimizingPlanner(std::make_shared<og::RRTConnect>(si_));  //no good
//    auto optimizingPlanner(std::make_shared<og::BiTRRT>(si_)); // last time use
//    auto optimizingPlanner(std::make_shared<og::PDST>(si_));  //no good
//        auto optimizingPlanner(std::make_shared<og::RRTstar>(si_)); // l
    og::InformedRRTstar* rrt = new og::InformedRRTstar(si_);
    rrt->setTreePruning(true);
    rrt->setRange(0.2);
    ob::PlannerPtr optimizingPlanner(rrt);

     // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef_);
    optimizingPlanner->setup();

    // print the settings for this space
    si_->printSettings(std::cout);

    // print the problem settings
    pdef_->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = optimizingPlanner->ob::Planner::solve(5.0);

    if (solved){

        std::cout << "Found solution:" << std::endl;
        ob::PathPtr path = pdef_->getSolutionPath();
        og::PathGeometric* pth;
        pth = pdef_->getSolutionPath()->as<og::PathGeometric>();
        //pth->printAsMatrix(std::cout);
        //print the path to screen
        //path->print(std::cout);
         og::PathSimplifier* pathBSpline = new og::PathSimplifier(si_);
        path_smooth_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*pdef_->getSolutionPath()));
        pathBSpline->smoothBSpline(*path_smooth_,3);
//        getStatesShow(path_smooth_);

//        vector<array<double,3>> joints_arr =  getStates(pth);
        vector<array<double,3>> cartesian_arr =  getStates(path_smooth_);

        // Clear memory
        pdef_->clearSolutionPaths();
        replan_flag_ = false;

        delete path_smooth_;
        return cartesian_arr;
    }
    else{
        ROS_ERROR("No solution found JM");
        return {};
    }
 }

vector<array<double,3>> SEARCHER_CARTESIAN::getStates( const ompl::geometric::PathGeometric* path){
    vector<array<double,3>> trajectory_points;
    array<double,3> trajectory_point{};

    for (size_t i = 0; i < path->getStateCount(); ++i){
        const ob::SE3StateSpace::StateType *se3state = path->getState(i)->as<ob::SE3StateSpace::StateType>();
        // extract the first component of the state and cast it to what we expect
        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        // extract the second component of the state and cast it to what we expect
        const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
        for(size_t j=0; j<3; j++){
            trajectory_point[j] = pos->values[j];
        }
        trajectory_points.push_back(trajectory_point);
    }
    return trajectory_points;
}



void SEARCHER_CARTESIAN::pubTrajectory(const std::array<double,7>& trajectory){
    trajectory_msgs::JointTrajectoryPoint j_p;
    trajectory_msgs::JointTrajectory j_traj;

    for(auto i=0; i<7; i++)
        j_p.positions.push_back(trajectory[i]);

    j_traj.points.push_back(j_p);
    trajPub_.publish(j_traj);
}
//
//ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si){
////    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
//    return ob::OptimizationObjectivePtr(new ob::MaximizeMinClearanceObjective(si));
// }