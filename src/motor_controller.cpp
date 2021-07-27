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

TMotorAK80_80 motor6("Right_Roll", 1);
TMotorAK10_9 motor7("Right_Pitch", 2);
TMotorAK60_6 motor8("Right_Slide", 4);
TMotorAK60_6 motor9("Right_Foot_Pitch", 3);
TMotorAK60_6 motor10("Right_Foot_Roll", 10);

// Set up managers
MotorManager left_leg_motors("can0");
MotorManager right_leg_motors("can1");

ros::Publisher leg_joint_publisher;

// Set postion goal subscriber
void setPositionGoal(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> data = msg->data;

    // Log the data within msg to ROS_INFO
    ROS_INFO("Setting position goal");
    
    // Left leg
    motor1.send_position_goal(data[0]);
    motor2.send_position_goal(data[1]);
    motor3.send_position_goal(data[2]);
    motor4.send_position_goal(data[3]);
    motor5.send_position_goal(data[4]);

    // Right leg
    motor6.send_position_goal(data[5]);
    motor7.send_position_goal(data[6]);
    motor8.send_position_goal(data[7]);
    motor9.send_position_goal(data[8]);
    motor10.send_position_goal(data[9]);

    // Update motor positions
    left_leg_motors.read_all();
    right_leg_motors.read_all();

    // Get joint states
    sensor_msgs::JointState leg_joint_states = left_leg_motors.get_joint_states();
    sensor_msgs::JointState right_leg_joint_states = right_leg_motors.get_joint_states();

    // Concatinate joint efforts
    leg_joint_states.effort.insert(
        leg_joint_states.effort.end(),
        std::make_move_iterator(right_leg_joint_states.effort.begin()),
        std::make_move_iterator(right_leg_joint_states.effort.end())
    );

    // Concatinate joint velocities
    leg_joint_states.velocity.insert(
        leg_joint_states.velocity.end(),
        std::make_move_iterator(right_leg_joint_states.velocity.begin()),
        std::make_move_iterator(right_leg_joint_states.velocity.end())
    );

    // Concatinate joint positions
    leg_joint_states.position.insert(
        leg_joint_states.position.end(),
        std::make_move_iterator(right_leg_joint_states.position.begin()),
        std::make_move_iterator(right_leg_joint_states.position.end())
    );

    // Concatinate joint names
    leg_joint_states.name.insert(
        leg_joint_states.name.end(),
        std::make_move_iterator(right_leg_joint_states.name.begin()),
        std::make_move_iterator(right_leg_joint_states.name.end())
    );

    // Set header
    leg_joint_states.header.stamp = ros::Time::now();

    // Publish joint states
    leg_joint_publisher.publish(leg_joint_states);
}

// Enable motors callback
bool enableMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Enable all motors");
    left_leg_motors.enable_all();
    right_leg_motors.enable_all();

    res.success = true;
    return true;
}

// Disable motors callback
bool disableMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Disable all motors");
    left_leg_motors.disable_all();
    right_leg_motors.enable_all();

    res.success = true;
    return true;
}

// Home motors callback
bool homeMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Homing all motors");
    left_leg_motors.home_all_individual(1.0);
    right_leg_motors.home_all_individual(1.0);
    
    res.success = true;
    return true;
}

// Main
int main(int argc, char **argv)
{
    // Adds all joints to the manager
    left_leg_motors.add_motor(&motor1);
    left_leg_motors.add_motor(&motor2);
    left_leg_motors.add_motor(&motor3);
    left_leg_motors.add_motor(&motor4);
    left_leg_motors.add_motor(&motor5);

    right_leg_motors.add_motor(&motor6);
    right_leg_motors.add_motor(&motor7);
    right_leg_motors.add_motor(&motor8);
    right_leg_motors.add_motor(&motor9);
    right_leg_motors.add_motor(&motor10);

    // Print out motor names
    left_leg_motors.print_all_motors();
    right_leg_motors.print_all_motors();

    // Connect to motors and run them to 0.
    ROS_INFO("Connecting to all motors");
    left_leg_motors.connect();
    left_leg_motors.enable_all();
    left_leg_motors.home_all_individual(1.0);


    // Set up left leg motor constants
    motor1.set_constants(2.0, 0.0);
    motor2.set_constants(2.0, 0.0);
    motor3.set_constants(5.0, 1.0);
    motor4.set_constants(5.0, 1.0);
    motor5.set_constants(5.0, 1.0);

    motor3.set_transmission_ratio(0.03);


    // Set up right leg motor constants
    motor6.set_constants(2.0, 0.0);
    motor7.set_constants(2.0, 0.0);
    motor8.set_constants(5.0, 1.0);
    motor9.set_constants(5.0, 1.0);
    motor10.set_constants(5.0, 1.0);

    motor8.set_transmission_ratio(0.03);

    ROS_INFO("Starting subscriber node");

    // Init the node
    ros::init(argc, argv, "leg_motor_controller");

    ros::NodeHandle n;

    // Add services
    ros::ServiceServer enableService = n.advertiseService("/motors/enable_all", enableMotors);
    ros::ServiceServer disableService = n.advertiseService("/motors/disable_all", disableMotors);
    ros::ServiceServer homeService = n.advertiseService("/motors/home_all", homeMotors);

    // Subscribe to the position goals topic
    ros::Subscriber sub = n.subscribe("/motors/position_goals", 1000, setPositionGoal);

    // Add publisher 
    leg_joint_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    // Spin the node
    ros::spin();

    return 0;
}