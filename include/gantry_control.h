#ifndef GANTRYCONTROL_H
#define GANTRYCONTROL_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <array>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include <nist_gear/VacuumGripperState.h>
#include <nist_gear/VacuumGripperControl.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include "utils.h"
#include <unordered_map>

#include <tf/transform_listener.h> //for shelves gap
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

class GantryControl {

  public:
    int shelf_1_gap;
    int shelf_2_gap;
    int shelf_3_gap;
    double shelf_4_x;
    double shelf_7_x;
    double shelf_10_x;
    int aisle_1_choice;
    int aisle_2_choice;
    int aisle_3_choice;
    int aisle_4_choice;
    void set_aisle_1_choice(int);
    void set_aisle_2_choice(int);
    void set_aisle_3_choice(int);
    void set_aisle_4_choice(int);
    int get_shelf_1_gap();
    int get_shelf_2_gap();
    int get_shelf_3_gap();
    std::vector<std::vector<double>> get_shelf_vector();
    void shelf_callback(std::string);

    GantryControl(ros::NodeHandle & node);
    void setRobotSpeed(double speed_factor, double acc_factor);
    void init();
    stats getStats(std::string function);
    Quat ToQuaternion(double roll, double pitch, double yaw);

//    bool moveGantry(std::string waypoints);
    void reachOut(geometry_msgs::Pose pose_end_effector);
//    bool pickPart(part part, std::string arm_name);
    bool pickPart(part part);
    bool pickMovingPart(part part);
    void placePart(part part, std::string agv);
    void placePart_right_arm(part part,std::string agv);
    /// Send command message to robot controller
    bool send_command(trajectory_msgs::JointTrajectory command_msg);
    void goToPresetLocation(PresetLocation location);

    void activateGripper(std::string gripper_id);
    void deactivateGripper(std::string gripper_id);
    nist_gear::VacuumGripperState getGripperState(std::string arm_name);
    geometry_msgs::Pose getTargetWorldPose(geometry_msgs::Pose target, std::string agv);
    geometry_msgs::Pose getTargetWorldPose_dummy(geometry_msgs::Pose target,
                                                                std::string agv);
    geometry_msgs::Pose getTargetWorldPose_right_arm(geometry_msgs::Pose target, std::string agv);

    //--preset locations;
    start start_, belt_pickup_;
    bin3 bin1_, bin2_, bin5_, bin6_, bin3_, bin4_, bin7_, bin8_, bin15_, bin16_, bin11_, bin12_, bin13_, bin14_, bin9_, bin10_, bin1_drop_, bin1_w1, bin1_w2;
    agv2 agv2_, agv1_;
    agv2_drop agv2_drop_, agv1_drop_;
    shelf5 shelf5_, shelf8_w1_, shelf8_w2_, shelf8_w3_, shelf8_w4_, shelf8_w5_,
            shelf8_w6_,shelf8_w7_, shelf8a_w1_, shelf8a_w2_, shelf8a_w3_, shelf8a_w4_,
            shelf8a_w5_, shelf8a_w6_,shelf11_w1_, shelf11_w2_, shelf11_w3_, shelf11_w4_
    ,shelf1_lb_w1, shelf1_lb_w2, shelf1_lb_w3, shelf1_lb_w4
    ,shelf1_lf_w1, shelf1_lf_w2, shelf1_lf_w3, shelf1_lf_w4
    ,shelf1_rb_w1, shelf1_rb_w2, shelf1_rb_w3, shelf1_rb_w4
    ,shelf1_rf_w1, shelf1_rf_w2, shelf1_rf_w3, shelf1_rf_w4
    ,shelf2_lb_w1, shelf2_lb_w2, shelf2_lb_w3, shelf2_lb_w4
    ,shelf2_lf_w1, shelf2_lf_w2, shelf2_lf_w3, shelf2_lf_w4
    ,shelf2_rb_w1, shelf2_rb_w2, shelf2_rb_w3, shelf2_rb_w4
    ,shelf2_rf_w1, shelf2_rf_w2, shelf2_rf_w3, shelf2_rf_w4
    ,shelf8_rb_w1,shelf8_rb_w2,shelf8_rb_w3,shelf8_rb_w4,shelf8_rb_w5,shelf8_rb_w6
    ,shelf8_lf_w1,shelf8_lf_w2,shelf8_lf_w3,shelf8_lf_w4,shelf8_lf_w5,shelf8_lf_w6, shelf8_lf_w7
    ,shelf8_lb_w1, shelf8_lb_w2, shelf8_lb_w3, shelf8_lb_w4, shelf8_lb_w5, shelf8_lb_w6
    ,shelf8_rf_w1,shelf8_rf_w2,shelf8_rf_w3,shelf8_rf_w4,shelf8_rf_w5,shelf8_rf_w6, shelf8_rf_w7
    ,shelf5_rb_w1,shelf5_rb_w2,shelf5_rb_w3,shelf5_rb_w4,shelf5_rb_w5,shelf5_rb_w6
    ,shelf5_lf_w1,shelf5_lf_w2,shelf5_lf_w3,shelf5_lf_w4,shelf5_lf_w5,shelf5_lf_w6, shelf5_lf_w7
    ,shelf5_lb_w1, shelf5_lb_w2, shelf5_lb_w3, shelf5_lb_w4, shelf5_lb_w5, shelf5_lb_w6
    ,shelf5_rf_w1,shelf5_rf_w2,shelf5_rf_w3,shelf5_rf_w4,shelf5_rf_w5,shelf5_rf_w6, shelf5_rf_w7
    ,shelf11_rb_w1,shelf11_rb_w2,shelf11_rb_w3,shelf11_rb_w4,shelf11_rb_w5,shelf11_rb_w6
    ,shelf11_lf_w1,shelf11_lf_w2,shelf11_lf_w3,shelf11_lf_w4,shelf11_lf_w5,shelf11_lf_w6, shelf11_lf_w7
    ,shelf11_lb_w1, shelf11_lb_w2, shelf11_lb_w3, shelf11_lb_w4, shelf11_lb_w5, shelf11_lb_w6
    ,shelf11_rf_w1,shelf11_rf_w2,shelf11_rf_w3,shelf11_rf_w4,shelf11_rf_w5,shelf11_rf_w6, shelf11_rf_w7,
    GAP_1,GAP_2,GAP_3,gap_1_1,gap_1_2,gap_2_2,gap_2_3,gap_3_3,gap_3_4;


    waypoint_1 waypoint_1_;
    waypoint_2 waypoint_2_;
    waypoint_3 waypoint_3_;
    waypoint_4 waypoint_4_;

    pose_change pose_change_1_agv1, pose_change_2_agv1,pose_change_1_agv2,pose_change_2_agv2;
    agv2_flip agv2_flip_,agv1_flip_;
    flip_target agv2_flip_target_,agv1_flip_target_;
    std::unordered_map<std::string, std::vector<PresetLocation>> pickup_locations;
    std::unordered_map<int, char>:: iterator itr;

  private:
    std::vector<double> joint_group_positions_;
    ros::NodeHandle node_;
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Options full_robot_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_arm_options_;
    moveit::planning_interface::MoveGroupInterface::Options left_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface::Options right_ee_link_options_;
    moveit::planning_interface::MoveGroupInterface full_robot_group_;
    moveit::planning_interface::MoveGroupInterface left_arm_group_;
    moveit::planning_interface::MoveGroupInterface right_arm_group_;
    moveit::planning_interface::MoveGroupInterface left_ee_link_group_;
    moveit::planning_interface::MoveGroupInterface right_ee_link_group_;

    double left_ee_roll_;
    double left_ee_pitch_;
    double left_ee_yaw_;
    std::array<float,4> left_ee_quaternion_;

    sensor_msgs::JointState current_joint_states_;


    nist_gear::VacuumGripperState current_left_gripper_state_;
    nist_gear::VacuumGripperState current_right_gripper_state_;

    control_msgs::JointTrajectoryControllerState current_gantry_controller_state_;
    control_msgs::JointTrajectoryControllerState current_left_arm_controller_state_;
    control_msgs::JointTrajectoryControllerState current_right_arm_controller_state_;

    ros::Publisher gantry_joint_trajectory_publisher_;
    ros::Publisher left_arm_joint_trajectory_publisher_;
    ros::Publisher right_arm_joint_trajectory_publisher_;

    ros::Subscriber joint_states_subscriber_;
    ros::Subscriber left_gripper_state_subscriber_;
    ros::Subscriber right_gripper_state_subscriber_;
    ros::Subscriber gantry_controller_state_subscriber_;
    ros::Subscriber left_arm_controller_state_subscriber_;
    ros::Subscriber right_arm_controller_state_subscriber_;

    ros::ServiceClient left_gripper_control_client;
    ros::ServiceClient right_gripper_control_client;

    // ---------- Callbacks ----------
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr & joint_state_msg);
    void left_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void right_gripper_state_callback(const nist_gear::VacuumGripperState::ConstPtr & msg);
    void gantry_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void left_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);
    void right_arm_controller_state_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr & msg);




     // collect stats
    stats init_;
    stats moveJ_;
    stats IK_;
    stats moveGantry_;
    stats pickPart_;
    stats placePart_;
    stats dropPart_;
    stats gripFirmly_;
    stats gripFromBelt_;
    stats grip_;
};

#endif
