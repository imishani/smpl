#include <sbpl_kdl_robot_model/kdl_robot_model.h>

#ifndef PUSHING_KDL_ROBOT_MODEL_H
#define PUSHING_KDL_ROBOT_MODEL_H

#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>

namespace smpl {

class InverseVelocityInterface : public virtual RobotModel
{
public:

    /// \brief Return the number of redundant joint variables.
    virtual bool computeInverseVelocity(
        const RobotState& jnt_positions,
        const std::vector<double>& cart_velocities,
        RobotState& jnt_velocities) = 0;
};

class PushingKDLRobotModel : public KDLRobotModel, public virtual InverseVelocityInterface
{
public:

    bool init(
        const std::string& robot_description,
        const std::string& base_link,
        const std::string& tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX);

    bool computeInverseVelocity(
        const RobotState& jnt_positions,
        const std::vector<double>& cart_velocities,
        RobotState& jnt_velocities);

    bool computeJacobian(
        const RobotState& jnt_positions,
        KDL::Jacobian& Jq);
    bool computeJacobian(
        const RobotState& jnt_positions,
        Eigen::MatrixXd& Jq);

    auto getExtension(size_t class_code) -> Extension* override;

private:

	std::unique_ptr<KDL::ChainIkSolverVel_pinv> m_cart_to_jnt_vel_solver;
    std::unique_ptr<KDL::ChainJntToJacSolver> m_Jq_solver;

};

} // namespace smpl

#endif
