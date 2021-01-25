//
// Created by jieming on 02.12.20.
//

#include "SEARCHER.h"
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

bool SEARCHER::isStateValid2(const ob::State *state){
    //add orientation constraints
    vector<double> joint_pos(7);

    for(size_t j=0; j<7; j++){
        joint_pos[j] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
    }
    // test orientation
    robot_.setJoints(Eigen::Map<Eigen::Vector7d>(joint_pos.data()), Eigen::Vector7d::Zero());
    Eigen::Affine3d trans = robot_.fkEE();
    Eigen::Matrix3d ori = trans.linear();
    Eigen::Matrix3d goal_ori;
    goal_ori << 0,0,1, 0,-1,0, 1,0,0;
    Eigen::Matrix3d error = ori.transpose() * goal_ori;
    if(error(0,0) > 1 || error(1,1)>1){
        return false;
    }
    // test collision
    return !collision_check_.checkCollision(joint_pos);
}

bool SEARCHER::isStateValid(const ob::State *state){
//    return true;
    vector<double> joint_pos(7);

    for(size_t j=0; j<7; j++){
        joint_pos[j] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
    }
     return !collision_check_.checkCollision(joint_pos);
 }

//public
SEARCHER::SEARCHER(){
//    client_ = n_.serviceClient<msg_pkg::Collisioncheck>("collision_check_srv");
//    traj_pub_ = n_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("waypoints",1);
//    vis_pub_ = n_.advertise<visualization_msgs::Marker>( "rrt_marker", 10 );
    trajPub_ =  n_.advertise<trajectory_msgs::JointTrajectory>("/robot1/jointTrajectory", 2);

    collision_check_.callJointStateCB = false;

    // Create the state space for the manipulator
    space_ = ob::StateSpacePtr(new ompl::base::RealVectorStateSpace(7));
    // create a start state
    ob::ScopedState<ob::RealVectorStateSpace> start(space_);
    // create a goal state
    ob::ScopedState<ob::RealVectorStateSpace> goal(space_);

    // Bound the joints of the manipulator between [-PI, PI]
    ob::RealVectorBounds bounds(7);
    bounds.setLow(0,-2.8973); bounds.setHigh(0,2.8973);
    bounds.setLow(1,-1.7628); bounds.setHigh(1,1.7628);
    bounds.setLow(2,-2.8973); bounds.setHigh(2,2.8973);
    bounds.setLow(3,-3.0718); bounds.setHigh(3,-0.0698);
    bounds.setLow(4,-2.8973); bounds.setHigh(4,2.8973);
    bounds.setLow(5,-0.0175); bounds.setHigh(5,3.7525);
    bounds.setLow(6,-2.8973); bounds.setHigh(6,2.8973);
    space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // construct an instance of  space information from this state space
    si_ = ob::SpaceInformationPtr(new ob::SpaceInformation(space_));
    si_->setStateValidityChecker(std::bind(&SEARCHER::isStateValid2, this, std::placeholders::_1 ));
    si_->setStateValidityCheckingResolution(0.02);

    si_->setup();
    // set state validity checking for this space
     start.random();
     goal.random();

    // create a problem instance
    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si_));
    // set the start and goal states
    pdef_->setStartAndGoalStates(start, goal);
   pdef_->setOptimizationObjective(getPathLengthObjWithCostToGo(si_));
//    pdef_->setOptimizationObjective(getPathLengthObjective(si_));

    std::cout << "initialized " << std::endl;
}
// Destructor
SEARCHER::~SEARCHER(){
 //    delete path_smooth_;
};

void SEARCHER::setStart(const std::array<double, 7>& start_joints){
    ob::ScopedState<ob::RealVectorStateSpace> start(space_);
    for(size_t i=0; i<7; i++){
        start[i] =  start_joints[i];
      }
    pdef_->clearStartStates();
    pdef_->addStartState(start);
    start.print();
}

void SEARCHER::setGoal(const std::array<double, 7>& goal_joints){
    ob::ScopedState<ob::RealVectorStateSpace> goal(space_);
    for(size_t i=0; i<7; i++)
        goal[i] = goal_joints[i];

    pdef_->clearGoal();
    pdef_->setGoalState(goal);

    cout << "set goal  ";  pdef_->getGoal()->print();

 }

void SEARCHER::replan(){
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

std::optional< vector<array<double,7>>> SEARCHER::plan(){
    int cnt=0;
    while(path_smooth_ == NULL && cnt<=2000) {
        cnt++;
//    auto optimizingPlanner(std::make_shared<og::InformedRRTstar>(si_));
//        auto optimizingPlanner(std::make_shared<og::RRTstar>(si_));
//        optimizingPlanner->setTreePruning(true);
//        optimizingPlanner->setRange(1);
//        optimizingPlanner->setRewireFactor(0.1);
//
//        optimizingPlanner->setGoalBias(0.05);
//        optimizingPlanner->setNumSamplingAttempts(5000);

//    optimizingPlanner->setNumSamplingAttempts(5000*1000);
        //    auto optimizingPlanner(std::make_shared<og::PRM>(si_));  //no good
//    auto optimizingPlanner(std::make_shared<og::RRTConnect>(si_));  //no good
//    auto optimizingPlanner(std::make_shared<og::BiTRRT>(si_)); // last time use
//    auto optimizingPlanner(std::make_shared<og::RRTConnect>(si_));  //no good
        auto optimizingPlanner(std::make_shared<og::PRM>(si_));  //no good


        // Set the problem instance for our planner to solve
        optimizingPlanner->setProblemDefinition(pdef_);
        optimizingPlanner->setup();


        // print the settings for this space
        si_->printSettings(std::cout);

        // print the problem settings
//        pdef_->print(std::cout);

        // attempt to solve the problem within one second of planning time
        ob::PlannerStatus solved = optimizingPlanner->ob::Planner::solve(10.0);
        if (solved) {

            std::cout << "Found solution:" << std::endl;
            ob::PathPtr path = pdef_->getSolutionPath();
            og::PathGeometric *pth;
            pth = pdef_->getSolutionPath()->as<og::PathGeometric>();
            pth->interpolate();
//        pth->printAsMatrix(std::cout);
            //print the path to screen
            //path->print(std::cout);
            og::PathSimplifier *pathBSpline = new og::PathSimplifier(si_);
            path_smooth_ = new og::PathGeometric(dynamic_cast<const og::PathGeometric &>(*pdef_->getSolutionPath()));
            pathBSpline->smoothBSpline(*path_smooth_, 5);

//        getStatesShow(path_smooth_);

//            vector<array<double, 7>> joints_arr = getStates(pth);
            vector<array<double,7>> joints_arr =  getStates(path_smooth_);

            // Clear memory
            pdef_->clearSolutionPaths();
            replan_flag_ = false;
            return joints_arr;

//            delete path_smooth_;
        } else {
            ROS_ERROR("No solution found JM");
            return {};
        }
    }


}

vector<array<double,7>> SEARCHER::getStates( const ompl::geometric::PathGeometric* path){
    vector<array<double,7>> trajectory_points;
    array<double,7> trajectory_point{};

    for (size_t i = 0; i < path->getStateCount(); ++i){
        const ob::State* state =  path->getState(i);

        for(size_t j=0; j<7; j++){
            trajectory_point[j] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
        }
        trajectory_points.push_back(trajectory_point);
    }
    return trajectory_points;
}

void SEARCHER::getStatesShow( const ompl::geometric::PathGeometric* path){
    ros::Rate rate(1);
    std::array<double,7> trajectory_point;
    for (size_t i = 0; i < path->getStateCount(); ++i){
        const ob::State* state =  path->getState(i);

        for(size_t j=0; j<7; j++){
            trajectory_point[j] = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j];
        }

        pubTrajectory(trajectory_point);
        cout << "publish "<< i << "times" << endl;
        rate.sleep();
    }
}

void SEARCHER::pubTrajectory(const std::array<double,7>& trajectory){
    trajectory_msgs::JointTrajectoryPoint j_p;
    trajectory_msgs::JointTrajectory j_traj;

    for(auto i=0; i<7; i++)
        j_p.positions.push_back(trajectory[i]);

    j_traj.points.push_back(j_p);
    trajPub_.publish(j_traj);
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si){
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
//    return ob::OptimizationObjectivePtr(new ob::MaximizeMinClearanceObjective(si));
 }