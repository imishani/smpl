#include <sbpl_kdl_robot_model/pushing_kdl_robot_model.h>

#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/stl/memory.h>

namespace smpl {

bool PushingKDLRobotModel::init(
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    if (!KDLRobotModel::init(
            robot_description, base_link, tip_link, free_angle))
    {
        return false;
    }

    m_cart_to_jnt_vel_solver = make_unique<KDL::ChainIkSolverVel_pinv>(m_chain);
    m_Jq_solver = make_unique<KDL::ChainJntToJacSolver>(m_chain);

    return true;
}

bool PushingKDLRobotModel::computeInverseVelocity(
        const RobotState& jnt_positions,
        const std::vector<double>& cart_velocities,
        RobotState& jnt_velocities)
{
    KDL::JntArray  q_(jnt_positions.size());
    KDL::JntArray  qdot_(jnt_positions.size());
    KDL::Twist     xdot_;
    jnt_velocities.resize(jnt_positions.size());

    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
    }

    xdot_.vel(0) = cart_velocities[0];
    xdot_.vel(1) = cart_velocities[1];
    xdot_.vel(2) = cart_velocities[2];
    if(cart_velocities.size() == 3)
    {
      xdot_.rot(0) = 0;
      xdot_.rot(1) = 0;
      xdot_.rot(2) = 0;
    }
    else
    {
      xdot_.rot(0) = cart_velocities[3];
      xdot_.rot(1) = cart_velocities[4];
      xdot_.rot(2) = cart_velocities[5];
    }

    if (!m_cart_to_jnt_vel_solver->CartToJnt(q_, xdot_, qdot_) < 0) {
        ROS_WARN("Failed to find inverse joint velocities");
        return false;
    }

    for (size_t i = 0; i < jnt_velocities.size(); ++i) {
        jnt_velocities[i] = qdot_(i);
    }
    return true;
}

bool PushingKDLRobotModel::computeJacobian(
    const RobotState& jnt_positions,
    KDL::Jacobian& Jq)
{
    KDL::JntArray  q_(jnt_positions.size());
    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
    }

    Jq.resize(m_chain.getNrOfJoints());
    int error = m_Jq_solver->JntToJac(q_, Jq);
    if (error < 0) {
        return false;
    }

    return true;
}

bool PushingKDLRobotModel::computeJacobian(
    const RobotState& jnt_positions,
    Eigen::MatrixXd& Jq)
{
    KDL::Jacobian Jq_KDL;
    if (this->computeJacobian(jnt_positions, Jq_KDL))
    {
        Jq = Jq_KDL.data;
        return true;
    }

    return false;
}

auto PushingKDLRobotModel::getExtension(size_t class_code) -> Extension*
{
    if (class_code == GetClassCode<InverseKinematicsInterface>()
        || class_code == GetClassCode<InverseVelocityInterface>()) return this;

    return URDFRobotModel::getExtension(class_code);
}

} // namespace smpl
