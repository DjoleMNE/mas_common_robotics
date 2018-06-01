/**
 * This is the ROS node class for the Hybrid Dynamics Solver.
 * It provides the connection between the ROS environment
 * and the Solver_Vereshchagin class.
 */

#ifndef MCR_HYBRID_DYNAMICS_SOLVER_HPP_
#define MCR_HYBRID_DYNAMICS_SOLVER_HPP_
#include <hd_solver_vereshchagin.hpp>
#include <motion_specification.hpp>
#include <brics_actuator/JointAccelerations.h>
#include <mcr_common_msgs/EndEffectorConstraint.h>

KDL::Solver_Vereshchagin *hd_solver_;

class hybrid_dynamics_solver
{

public:

    hybrid_dynamics_solver();
    virtual ~hybrid_dynamics_solver();

    void jointstateCallback(sensor_msgs::JointStateConstPtr joints);

    static void cartAccCallback(mcr_common_msgs::EndEffectorConstraint desiredConstraint);
    static void extForceEECallback(geometry_msgs::Wrench externalForceEE);

    void init_hd_solver();

    void init_joint_msgs();

    void publishJointAccelerations(KDL::JntArray &joint_accelerations);
    void publishJointTorques(KDL::JntArray &joint_torques);

    KDL::Chain arm_chain_;
    std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;
    std::vector<bool> joint_positions_initialized;

    const int NUMBER_OF_CONSTRAINTS = 6;

    brics_actuator::JointAccelerations joint_acc_msg_;
    ros::Publisher joint_acc_publisher_;

    motion_specification motion_;

};

#endif /* MCR_HYBRID_DYNAMICS_SOLVER_HPP_*/
