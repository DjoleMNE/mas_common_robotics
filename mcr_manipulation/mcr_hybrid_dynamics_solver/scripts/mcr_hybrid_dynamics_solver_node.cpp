#include <mcr_manipulation_utils/ros_urdf_loader.h>
#include <kdl/kdl.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <ros/ros.h>
#include <hd_solver_vereshchagin.hpp>
#include <motion_specification.hpp>
#include <brics_actuator/JointAccelerations.h>
#include <mcr_common_msgs/EndEffectorConstraint.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <hd_solver_vereshchagin.hpp>
#include <brics_actuator/JointVelocities.h>
// #include <mcr_hybrid_dynamics_solver.hpp>
#include <cmath>
#include <stdlib.h>     /* abs */

KDL::Chain arm_chain_;
motion_specification motion_;
std::vector<boost::shared_ptr<urdf::JointLimits> > joint_limits_;
std::vector<bool> joint_positions_initialized;

const int NUMBER_OF_CONSTRAINTS = 6;

brics_actuator::JointAccelerations joint_acc_msg_;
ros::Publisher joint_acc_publisher_;

ros::Publisher joint_vel_publisher;

brics_actuator::JointVelocities joint_vel_msg_;

void init_hd_solver(){

    int number_of_segments = arm_chain_.getNrOfSegments();
    int number_of_joints = arm_chain_.getNrOfJoints();

    motion_.set_motion(number_of_joints,
                        number_of_segments,
                        NUMBER_OF_CONSTRAINTS);

    // external forces on the arm (Not including tool segment)
    for (int i = 0; i < number_of_segments - 1; i++) {
        KDL::Wrench externalForce(
           KDL::Vector(0.0, 0.0, 0.0), //Linear Force
           KDL::Vector(0.0, 0.0, 0.0)); //Torque
        motion_.external_force[i] = externalForce;
    }

    for (int i = 0; i < number_of_joints; i++) motion_.q(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) motion_.qd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) motion_.qdd(i) = 0.0;
    for (int i = 0; i < number_of_joints; i++) motion_.feedforward_torque(i) = 0.0;
}

void init_joint_msgs()
{
    joint_positions_initialized.resize(arm_chain_.getNrOfJoints(), false);
    joint_vel_msg_.velocities.resize(arm_chain_.getNrOfJoints());

    joint_acc_msg_.accelerations.resize(arm_chain_.getNrOfJoints());

    for (unsigned int i = 0; i < arm_chain_.getNrOfSegments(); i++)
    {
        joint_vel_msg_.velocities[i].joint_uri =
            arm_chain_.getSegment(i).getJoint().getName();
        joint_vel_msg_.velocities[i].unit = "s^-1 rad";

        joint_acc_msg_.accelerations[i].joint_uri =
            arm_chain_.getSegment(i).getJoint().getName();
        joint_acc_msg_.accelerations[i].unit = "s^-2 rad";
    }
}

void jointstateCallback(const sensor_msgs::JointStateConstPtr &joints)
{
    // std::cout << "s" << '\n';
    for (unsigned i = 0; i < joints->position.size(); i++)
    {

        const char* joint_uri = joints->name[i].c_str();

        for (unsigned int j = 0; j < arm_chain_.getNrOfJoints(); j++)
        {
            const char* chainjoint =
                arm_chain_.getSegment(j).getJoint().getName().c_str();

            //Compare names of current joint
            if (chainjoint != 0 && strcmp(chainjoint, joint_uri) == 0)
            {
                motion_.q.data[j] = joints->position[i];
                motion_.qd.data[j] = joints->velocity[i];
                joint_positions_initialized[j] = true;
            }
        }
    }
}

void cartAccCallback(mcr_common_msgs::EndEffectorConstraint &desiredConstraint)
{
    // for (size_t i = 0; i < joint_positions_initialized.size(); i++)
    // {
    //     if (!joint_positions_initialized[i])
    //     {
    //         std::cout << "joints not initialized" << std::endl;
    //         return;
    //     }
    // }

    motion_.end_effector_acceleration_energy_setpoint(0) = desiredConstraint.linear_acc.x;
    motion_.end_effector_acceleration_energy_setpoint(1) = desiredConstraint.linear_acc.y;
    motion_.end_effector_acceleration_energy_setpoint(2) = desiredConstraint.linear_acc.z;

    motion_.end_effector_acceleration_energy_setpoint(3) = desiredConstraint.angular_acc.x;
    motion_.end_effector_acceleration_energy_setpoint(4) = desiredConstraint.angular_acc.y;
    motion_.end_effector_acceleration_energy_setpoint(5) = desiredConstraint.angular_acc.z;

    KDL::Twist unit_constraint_force_linear_x(
            KDL::Vector((desiredConstraint.linear_force.x ? 1.0 : 0.0), 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_linear_x);

    KDL::Twist unit_constraint_force_linear_y(
            KDL::Vector(0.0, (desiredConstraint.linear_force.y ? 1.0 : 0.0), 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_linear_y);

    KDL::Twist unit_constraint_force_linear_z(
            KDL::Vector(0.0, 0.0, (desiredConstraint.linear_force.z ? 1.0 : 0.0)),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_linear_z);

    KDL::Twist unit_constraint_force_angular_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector((desiredConstraint.angular_force.x ? 1.0 : 0.0), 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_angular_x);

    KDL::Twist unit_constraint_force_angular_y(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, (desiredConstraint.angular_force.y ? 1.0 : 0.0), 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_angular_y);

    KDL::Twist unit_constraint_force_angular_z(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, (desiredConstraint.angular_force.z ? 1.0 : 0.0)));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_angular_z);
}

void extForceEECallback(geometry_msgs::Wrench externalForce)
{
    // for (size_t i = 0; i < joint_positions_initialized.size(); i++)
    // {
    //     if (!joint_positions_initialized[i])
    //     {
    //         std::cout << "joints not initialized" << std::endl;
    //         return;
    //     }
    // }

    int number_of_segments = arm_chain_.getNrOfSegments();

    KDL::Wrench externalForceEE(
        KDL::Vector(externalForce.force.x,
                    externalForce.force.y,
                    externalForce.force.z), //Force

        KDL::Vector(externalForce.torque.x,
                    externalForce.torque.z,
                    externalForce.torque.y));//Torque

    motion_.external_force[number_of_segments - 1] = externalForceEE;
    // std::cout << motion_.external_force[number_of_segments - 1] << '\n';
}

void publishJointAccelerations(KDL::JntArray &joint_accelerations)
{

    for (unsigned int i = 0; i < joint_accelerations.rows(); i++)
    {
        joint_acc_msg_.accelerations[i].value = joint_accelerations(i);

        ROS_DEBUG("%s: %.5f %s", joint_acc_msg_.accelerations[i].joint_uri.c_str(),
                  joint_acc_msg_.accelerations[i].value, joint_acc_msg_.accelerations[i].unit.c_str());

        if (std::isnan(joint_acc_msg_.accelerations[i].value))
        {
            ROS_ERROR("Invalid joint acceleration: nan");
            return;
        }
        if (std::fabs(joint_acc_msg_.accelerations[i].value) > 0.5)
        {
            ROS_ERROR("Invalid joint acceleration: too fast");
            return;
        }
    }
    joint_acc_publisher_.publish(joint_acc_msg_);
}

// void hybrid_dynamic_solver::publishJointTorques(KDL::JntArray &joint_torques)
// {
//
//     for (unsigned int i = 0; i < joint_accelerations.rows(); i++)
//     {
//         jointMsg.velocities[i].value = joint_accelerations(i);
//         ROS_DEBUG("%s: %.5f %s", jointMsg.velocities[i].joint_uri.c_str(),
//                   jointMsg.velocities[i].value, jointMsg.velocities[i].unit.c_str());
//         if (isnan(jointMsg.velocities[i].value))
//         {
//             ROS_ERROR("invalid joint velocity: nan");
//             return;
//         }
//         if (fabs(jointMsg.velocities[i].value) > 1.0)
//         {
//             ROS_ERROR("invalid joint velocity: too fast");
//             return;
//         }
//     }
//     joint_acc_publisher_.publish(jointMsg);
// }

void integrate_joints(KDL::JntArray &joint_data,
                      KDL::JntArray &integrated_data,
                      double dt)
{

    integrated_data.data = joint_data.data*dt;

    if (integrated_data.data.norm() <= 0.01)
        integrated_data.data.Zero(arm_chain_.getNrOfJoints());

}

void publishJointVelocities(KDL::JntArray& joint_velocities)
{
    for (unsigned int i = 0; i < joint_velocities.rows(); i++)
    {
        joint_vel_msg_.velocities[i].value = joint_velocities(i);
        ROS_DEBUG("%s: %.5f %s",
                  joint_vel_msg_.velocities[i].joint_uri.c_str(),
                  joint_vel_msg_.velocities[i].value,
                  joint_vel_msg_.velocities[i].unit.c_str());

        if (std::isnan(joint_vel_msg_.velocities[i].value))
        {
            ROS_ERROR("invalid joint velocity: nan");
            return;
        }
        if (std::fabs(joint_vel_msg_.velocities[i].value) > 1.0)
        {
            ROS_ERROR("invalid joint velocity: too fast");
            return;
        }
    }
    joint_vel_publisher.publish(joint_vel_msg_);
}

void stopMotion()
{
    for (unsigned int i = 0; i < joint_vel_msg_.velocities.size(); i++)
    {
        joint_vel_msg_.velocities[i].value = 0.0;

    }
    joint_vel_publisher.publish(joint_vel_msg_);
}

int main(int argc, char **argv)
{
    // hybrid_dynamics_solver solver_node;
    ros::init(argc, argv, "mcr_hybrid_dynamics_solver");
    ros::NodeHandle node_handle("~");

    //TODO:read from param
    std::string joint_effort_topic = "/joint_effort";
    std::string joint_state_topic = "/joint_states";
    std::string end_effector_acceleration_topic = "/end_effector_acceleration";
    std::string end_effector_force_topic = "/end_effector_force";
    std::string joint_velocity_topic = "joint_velocity";

    std::string root_name = "";
    std::string tooltip_name = "";

    if (!node_handle.getParam("root_name", root_name))
    {
        ROS_ERROR("No parameter for root_name specified");
        return -1;
    }

    ROS_INFO("Using %s as chain root [param: root_name]", root_name.c_str());

    if (!node_handle.getParam("tip_name", tooltip_name))
    {
        ROS_ERROR("No parameter for tip_name specified");
        return -1;
    }
    ROS_INFO("Using %s as tool tip [param: tip_name]", tooltip_name.c_str());


    //load URDF model
    ROS_URDF_Loader loader;
    loader.loadModel(node_handle,
                     root_name,
                     tooltip_name,
                     arm_chain_,
                     joint_limits_);

    int number_of_segments = arm_chain_.getNrOfSegments();
    int number_of_joints = arm_chain_.getNrOfJoints();


    //arm root acceleration
    KDL::Vector linearAcc(0.0, 0.0, -9.81); //gravitational acceleration along Z
    KDL::Vector angularAcc(0.0, 0.0, 0.0);
    KDL::Twist root_acc(linearAcc, angularAcc);

    KDL::Solver_Vereshchagin hd_solver_(arm_chain_,
                                       root_acc,
                                       NUMBER_OF_CONSTRAINTS);

    init_hd_solver();

    init_joint_msgs();

    //register publisher
    joint_vel_publisher = node_handle.advertise<brics_actuator::JointVelocities>(
                            joint_velocity_topic, 1);
    // joint_effort_publisher_ = node_handle.advertise<brics_actuator::JointAccelerations>(
    //                         joint_effort_topic, 1);
    //

    //register subscriber
    // ros::Subscriber sub_joint_states = node_handle.subscribe(joint_state_topic,
    //     1, jointstateCallback);

    //
    // ros::Subscriber sub_acc = node_handle.subscribe(end_effector_acceleration_topic,
    //                                                 1,cartAccCallback);
    //
    // ros::Subscriber sub_force = node_handle.subscribe(end_effector_force_topic,
    //                                                   1, extForceEECallback);

    //loop with 50Hz
    double rate = 1000;
    ros::Rate loop_rate(rate);

    //navigation
    motion_.q(0) = 2.12019;
    motion_.q(1) = 0.075952;
    motion_.q(2) = -1.53240;
    motion_.q(3) = 3.35214;
    motion_.q(4) = 2.93816;

    //shelf_intermediate
    // motion_.q(0) = 3.728617;
    // motion_.q(1) = 0.087803;
    // motion_.q(2) = -1.484166;
    // motion_.q(3) = 3.35212;
    // motion_.q(4) = 2.957057;

    //Pregrasp pose
    // motion_.q(0) = 2.2108;
    // motion_.q(1) = 1.77536;
    // motion_.q(2) = -1.68529;
    // motion_.q(3) = 3.40588;
    // motion_.q(4) = 2.93889;

    //Folded pose
    // motion_.q(0) = -0.0;
    // motion_.q(1) = -0.0;
    // motion_.q(2) = 0.0;
    // motion_.q(3) = 2.212389e-05;
    // motion_.q(4) = 0.001438;

    //Stand still command
    KDL::Twist unit_constraint_force_x(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(0, unit_constraint_force_x);
    motion_.end_effector_acceleration_energy_setpoint(0) = 0.0;

    KDL::Twist unit_constraint_force_y(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(1, unit_constraint_force_y);
    motion_.end_effector_acceleration_energy_setpoint(1) = 0.0;

    KDL::Twist unit_constraint_force_z(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(2, unit_constraint_force_z);
    motion_.end_effector_acceleration_energy_setpoint(2) = 0.0;
    //
    KDL::Twist unit_constraint_force_x1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(1.0, 0.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(3, unit_constraint_force_x1);
    motion_.end_effector_acceleration_energy_setpoint(3) = 0.0;

    KDL::Twist unit_constraint_force_y1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 1.0, 0.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(4, unit_constraint_force_y1);
    motion_.end_effector_acceleration_energy_setpoint(4) = 0.0;

    KDL::Twist unit_constraint_force_z1(
            KDL::Vector(0.0, 0.0, 0.0),     // linear
            KDL::Vector(0.0, 0.0, 1.0));    // angular
    motion_.end_effector_unit_constraint_forces.setColumn(5, unit_constraint_force_z1);
    motion_.end_effector_acceleration_energy_setpoint(5) = 0.0;

    // std::cout << motion_.end_effector_acceleration_energy_setpoint << '\n';

    KDL::Wrench externalForceEE(KDL::Vector(50.0,
                                            0.0,
                                            0.0), //Linear Force
                                KDL::Vector(0.0,
                                            0.0,
                                            0.0)); //Torque

    motion_.external_force[number_of_segments - 1] = externalForceEE;

    int result = hd_solver_.CartToJnt(motion_.q,
                                      motion_.qd,
                                      motion_.qdd, //qdd_ is overwritten by resulting acceleration
                                      motion_.end_effector_unit_constraint_forces,       // alpha
                                      motion_.end_effector_acceleration_energy_setpoint, // beta
                                      motion_.external_force,
                                      motion_.feedforward_torque);
    std::cout << "Solver return: "<< result << '\n';
    std::cout <<"Joints  Acc: "<< motion_.qdd << '\n';
    assert(result == 0);

    //Time sampling interval
    double dt = 1.0 / rate;

    double max_joint_vel = 0.25; // radian/s

    KDL::JntArray joint_velocities(arm_chain_.getNrOfJoints());
    integrate_joints(motion_.qdd, joint_velocities, dt);

    std::cout << "Joint Vel:: " << joint_velocities <<'\n';

    std::vector<KDL::Twist> frame_acceleration_;
    frame_acceleration_.resize(arm_chain_.getNrOfSegments()+1);
    hd_solver_.get_transformed_link_acceleration(frame_acceleration_);

    std::cout << "Frame ACC" << '\n';
    for (size_t i = 0; i < arm_chain_.getNrOfSegments()+1; i++) 
    {
        std::cout << frame_acceleration_[i] << '\n';
    }

    // Eigen::VectorXd nu(NUMBER_OF_CONSTRAINTS, 1);
    // hd_solver_.get_constraint_magnitude(nu);
    // std::cout << nu << '\n';

    stopMotion();
    ros::Duration(0.5).sleep();

    publishJointVelocities(joint_velocities);

    ros::Duration(1.2).sleep();
    stopMotion();

    // while (ros::ok())
    // {
    //     ros::spinOnce();
        
    //     publishJointVelocities(joint_velocities);

    //     ros::Duration(0.5).sleep();
    //     stopMotion();
    //     // publishJointAccelerations(motion_.qdd);
    
    //     // if (watchdog())
    //     // {
    //     // }
    
    //     // std::cout << motion_.q << '\n';
    //     // loop_rate.sleep();
    // }

    return 0;
}
