#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Trigger.h"

#include "tmotor_controller/MotorManager.h"

#include "tmotor_controller/TMotor.h"
#include "tmotor_controller/TMotorAK80_80.h"
#include "tmotor_controller/TMotorAK60_6.h"
#include "tmotor_controller/TMotorAK10_9.h"

#include <string>

// Set up motors
TMotorAK80_80 motor1("Left_Roll", 1);
TMotorAK10_9 motor2("Left_Pitch", 2);
TMotorAK60_6 motor3("Left_Slide", 4);
TMotorAK60_6 motor4("Left_Foot_Pitch", 3);
TMotorAK60_6 motor5("Left_Foot_Roll", 10);

// Set up managers
MotorManager manager("can0");

ros::Publisher leg_1_joint_publisher;

// Set postion goal subscriber
void setPositionGoal(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> data = msg->data;

    // Log the data within msg to ROS_INFO
    ROS_INFO("Setting position goal");
    
    motor1.send_position_goal(data[0]);
    motor2.send_position_goal(data[1]);
    motor3.send_position_goal(data[2]);
    motor4.send_position_goal(data[3]);
    motor5.send_position_goal(data[4]);

    // Update motor positions
    manager.read_all();

    sensor_msgs::JointState joint_state = manager.get_joint_states();

    joint_state.header.stamp = ros::Time::now();

    leg_1_joint_publisher.publish(joint_state);
}

// Enable motors callback
bool enableMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Enable all motors");
    manager.enable_all();

    res.success = true;
    return true;
}

// Disable motors callback
bool disableMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Disable all motors");
    manager.disable_all();

    res.success = true;
    return true;
}

// Disable motors callback
bool homeMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Homing all motors");
    manager.home_all_individual(1.0);
    
    res.success = true;
    return true;
}

// Main
int main(int argc, char **argv)
{
    // Adds each joint to the manager
    manager.add_motor(&motor1);
    manager.add_motor(&motor2);
    manager.add_motor(&motor3);
    manager.add_motor(&motor4);
    manager.add_motor(&motor5);

    manager.print_all_motors();

    ROS_INFO("Connecting to all motors");
    manager.connect();
    manager.enable_all();
    manager.home_all_individual(1.0);

    motor1.set_constants(2.0, 0.0);
    motor2.set_constants(2.0, 0.0);
    motor3.set_constants(5.0, 1.0);
    motor4.set_constants(5.0, 1.0);
    motor5.set_constants(5.0, 1.0);

    motor3.set_transmission_ratio(0.03);

    ROS_INFO("Starting subscriber node");

    // Init the node
    ros::init(argc, argv, "leg_motor_controller");

    ros::NodeHandle n;

    // Add services
    ros::ServiceServer enableService = n.advertiseService("/motors/enable_all", enableMotors);
    ros::ServiceServer disableService = n.advertiseService("/motors/disable_all", disableMotors);
    ros::ServiceServer homeService = n.advertiseService("/motors/home_all", homeMotors);

    // Subscribe to the /motors/enable topic
    ros::Subscriber sub = n.subscribe("/motors/set_position_goals", 1000, setPositionGoal);

    // Add publisher 
    leg_1_joint_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    // Spin the node
    ros::spin();

    return 0;
}