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
TMotorAK80_80 motor1("Pitch", 1);
TMotorAK10_9 motor2("Roll", 2);
TMotorAK60_6 motor3("Slide", 3);
TMotorAK60_6 motor4("Ankle 1", 4);
TMotorAK60_6 motor5("Ankle 2", 10);

// Set up managers
MotorManager manager("can0");

ros::Publisher leg_1_pub;

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

    leg_1_pub.publish(joint_state);
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

    motor1.set_constants(1.0, 0.5);
    motor2.set_constants(1.0, 0.5);
    motor3.set_constants(20.0, 0.5);
    motor4.set_constants(20.0, 0.5);
    motor5.set_constants(20.0, 0.5);

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
    leg_1_pub = n.advertise<sensor_msgs::JointState>("/motors/position", 1000);

    // Spin the node
    ros::spin();

    return 0;
}