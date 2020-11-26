#ifndef COMPETITION_H
#define COMPETITION_H

#include <vector>

#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include <nist_gear/Order.h>
#include <nist_gear/LogicalCameraImage.h>
#include <nist_gear/Proximity.h>

#include <tf/transform_listener.h> //for shelves gap
#include <tf/LinearMath/Vector3.h> //for shelves gap

#include "utils.h"

/**
 * @brief Competition class
 * 
 */
class Competition
{
public:
    //--methods
    explicit Competition(ros::NodeHandle & node);
    void init();
    void startCompetition();
    void endCompetition();
    void competition_state_callback(const std_msgs::String::ConstPtr & msg);
    void logical_camera_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg, int cam_idx);
    void quality_control_sensor_1_subscriber_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
    void quality_control_sensor_2_subscriber_callback(const nist_gear::LogicalCameraImage::ConstPtr & msg);
    std::array<std::array<part, 20>, 20> get_parts_from_camera();
    part get_quality_sensor_status_agv2();
    part get_quality_sensor_status_agv1();
    std::vector<std::vector<std::vector<master_struct> > > get_master_vector();
    void print_parts_detected();
    void print_parts_to_pick();
    void pre_kitting();
    void during_kitting(part);
    void competition_clock_callback(const rosgraph_msgs::Clock::ConstPtr & msg);
    void order_callback(const nist_gear::Order::ConstPtr & msg);
    double getClock();
    double getStartTime();
    std::string getCompetitionState();
    stats getStats(std::string function);
    std::vector<nist_gear::Order> get_received_order_vector();
    void setter_delivered(int i, int j, int k);
    void delete_completed_order(int i);
    std::vector<part> parts_from_15_camera;
    std::vector <part> get_parts_from_15_camera();
    std::array <part, 20> get_parts_from_16_camera();
    std::array <part, 20> get_parts_from_17_camera();

    //--attributes
    std::array<part, 20> parts_from_11_camera;
//    std::array<part, 20> parts_from_15_camera;
    std::array<part, 20> parts_from_16_camera;
    std::array<part, 20> parts_from_17_camera;
    bool conveyor_belt_part_status = false;

    // all breakbeam related initialization
    void breakbeam_sensor_0_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_1_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_11_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_12_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_13_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_14_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_15_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_16_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_21_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_22_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_23_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_24_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_25_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_26_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_31_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_32_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_33_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_34_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_35_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_36_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_41_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_42_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_43_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_44_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_45_callback(const nist_gear::Proximity::ConstPtr & msg);
    void breakbeam_sensor_46_callback(const nist_gear::Proximity::ConstPtr & msg);

    bool breakbeam_conveyor_belt_part_status_0= false;
    bool breakbeam_conveyor_belt_part_status_1= false;
    bool breakbeam_part_status_11= false;
    bool breakbeam_part_status_12= false;
    bool breakbeam_part_status_13= false;
    bool breakbeam_part_status_14= false;
    bool breakbeam_part_status_15= false;
    bool breakbeam_part_status_16= false;
    bool breakbeam_part_status_21= false;
    bool breakbeam_part_status_22= false;
    bool breakbeam_part_status_23= false;
    bool breakbeam_part_status_24= false;
    bool breakbeam_part_status_25= false;
    bool breakbeam_part_status_26= false;
    bool breakbeam_part_status_31= false;
    bool breakbeam_part_status_32= false;
    bool breakbeam_part_status_33= false;
    bool breakbeam_part_status_34= false;
    bool breakbeam_part_status_35= false;
    bool breakbeam_part_status_36= false;
    bool breakbeam_part_status_41= false;
    bool breakbeam_part_status_42= false;
    bool breakbeam_part_status_43= false;
    bool breakbeam_part_status_44= false;
    bool breakbeam_part_status_45= false;
    bool breakbeam_part_status_46= false;

    int get_human_existence();
    int human_detected = 0;

private:
    ros::NodeHandle node_;

    std::string competition_state_;
    double current_score_;
    ros::Time competition_clock_;
    double competition_start_time_; // wall time in sec

    ros::Subscriber current_score_subscriber_;
    ros::Subscriber competition_state_subscriber_;
    ros::Subscriber competition_clock_subscriber_;
    ros::Subscriber orders_subscriber_;
    ros::Subscriber quality_control_sensor_1_subscriber_;
    ros::Subscriber quality_control_sensor_2_subscriber_;
    std::vector<nist_gear::Order> received_orders_;


    //BREAK_BEAM subscribers

    ros::Subscriber breakbeam_sensor_0_subscriber_;
    ros::Subscriber breakbeam_sensor_1_subscriber_;
    ros::Subscriber breakbeam_sensor_11_subscriber_;
    ros::Subscriber breakbeam_sensor_12_subscriber_;
    ros::Subscriber breakbeam_sensor_13_subscriber_;
    ros::Subscriber breakbeam_sensor_14_subscriber_;
    ros::Subscriber breakbeam_sensor_15_subscriber_;
    ros::Subscriber breakbeam_sensor_16_subscriber_;
    ros::Subscriber breakbeam_sensor_21_subscriber_;
    ros::Subscriber breakbeam_sensor_22_subscriber_;
    ros::Subscriber breakbeam_sensor_23_subscriber_;
    ros::Subscriber breakbeam_sensor_24_subscriber_;
    ros::Subscriber breakbeam_sensor_25_subscriber_;
    ros::Subscriber breakbeam_sensor_26_subscriber_;
    ros::Subscriber breakbeam_sensor_31_subscriber_;
    ros::Subscriber breakbeam_sensor_32_subscriber_;
    ros::Subscriber breakbeam_sensor_33_subscriber_;
    ros::Subscriber breakbeam_sensor_34_subscriber_;
    ros::Subscriber breakbeam_sensor_35_subscriber_;
    ros::Subscriber breakbeam_sensor_36_subscriber_;
    ros::Subscriber breakbeam_sensor_41_subscriber_;
    ros::Subscriber breakbeam_sensor_42_subscriber_;
    ros::Subscriber breakbeam_sensor_43_subscriber_;
    ros::Subscriber breakbeam_sensor_44_subscriber_;
    ros::Subscriber breakbeam_sensor_45_subscriber_;
    ros::Subscriber breakbeam_sensor_46_subscriber_;

    // to collect statistics
    stats init_;
};

#endif
