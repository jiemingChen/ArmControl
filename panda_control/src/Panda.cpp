//
// Created by jieming on 04.11.20.
//

#include "Panda.h"

Panda::Panda(){
    urdf_file_ = "/home/jieming/catkin_ws/src/panda_simulation/franka_description/robots/model_hand.urdf";
    base_link_ = "panda_link0";
    tip_link_  = "virtual_link";//virtual_link

    if (!kdl_parser::treeFromFile(urdf_file_, tree_))
        perror("Failed to construct kdl tree\n");

    if (!tree_.getChain(base_link_, tip_link_, chain_))
        printf("Couldn't find chain from %s to %s\n", base_link_.c_str(), tip_link_.c_str());

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
    jnt_to_dyn_.reset(new KDL::ChainDynParam(chain_, KDL::Vector(0, 0, -9.8)));
    num_joints_  = chain_.getNrOfJoints();
    q_.resize(num_joints_);
    q0_.resize(num_joints_);
    qdot_.resize(num_joints_);
    J_.resize(num_joints_);
    tau_.resize(num_joints_);
    gravity_torque_.resize(num_joints_);
    coriolis_torque_.resize(num_joints_);
    D_.resize(num_joints_);
    q_min_.resize(num_joints_);
    q_max_.resize(num_joints_);

    Eigen::Vector7d qmin, qmax;
    qmax << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    qmin << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
    q_min_.data = qmin;
    q_max_.data = qmax;
#if 1
    ik_slover_.reset(new TRAC_IK::TRAC_IK(chain_,q_min_,q_max_));
//    ik_slover_.reset(new TRAC_IK::TRAC_IK(base_link_, tip_link_, urdf_file_));
    if(!ik_slover_->getKDLChain(chain_))
        perror("no valid chain");
    KDL::JntArray ll, ul;
    if(!ik_slover_->getKDLLimits(ll, ul))
        perror("no valid joint limits");
#endif

}

void Panda::setJoints(const sensor_msgs::JointState::ConstPtr& msg){
    for(unsigned int i=0;i<num_joints_;i++){
        q_(i) = msg->position[i];
        qdot_(i) = msg->velocity[i];
    }
}

void Panda::setJoints(const Eigen::Matrix<double, 7, 1>& pos, const Eigen::Matrix<double, 7, 1>& vel){
    for(unsigned int i=0;i<num_joints_;i++){
        q_(i) = pos[i];
        qdot_(i) = vel[i];
    }
//    dyn();
}


Eigen::Affine3d Panda::fkEE(){
    // Create the frame that will contain the results
    KDL::Frame cartpos;

    // Calculate forward position kinematics
    bool kinematics_status = jnt_to_pose_solver_->JntToCart(q_, cartpos);

    Eigen::Affine3d O_EE;

    if(kinematics_status>=0){
         tf::transformKDLToEigen(cartpos, O_EE);
        // cout << O_EE.matrix() << endl;
    }
    else{
        perror("%s \n Error: could not calculate forward kinematics :(");
    }
    return O_EE;
}

Eigen::Vector7d Panda::iK(Eigen::Affine3d cart_pos){
    KDL::Frame cartpos;
    tf::transformEigenToKDL(cart_pos, cartpos);
    KDL::JntArray q_res;
    int rc= ik_slover_->CartToJnt(q_, cartpos, q_res);
    if(rc < 0)
        printf("%s \n","Error: could not calculate inverse kinematics :(");

    Eigen::Vector7d res;
    for(auto i=0; i<7; i++)
        res(i) = q_res(i);
    return res;
}


void Panda::dyn(){
    jnt_to_dyn_->JntToMass(q_, D_);
    jnt_to_dyn_->JntToGravity(q_, gravity_torque_);
    jnt_to_dyn_->JntToCoriolis(q_, qdot_, coriolis_torque_);
}

Eigen::VectorXd  Panda::gravityTorque(){
    return gravity_torque_.data;
}

Eigen::VectorXd Panda::coriolis(){
    return coriolis_torque_.data;
}

Eigen::MatrixXd Panda::mass(){
    return D_.data;
}


Eigen::MatrixXd Panda::jacobian(){
    jnt_to_jac_solver_->JntToJac(q_, J_);
    return J_.data;
}

//else{
//for(unsigned int i = 0; i < num_joints_; i++)
//std::cout << q_res(i) << " ";
//}