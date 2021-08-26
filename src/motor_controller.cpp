// written by t.gotem, y.cheng, o.helander, 
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

#include <math.h>

// Set up motors
TMotorAK80_80 motor1("Right_Roll", 1);
TMotorAK10_9 motor2("Right_Pitch", 2);
TMotorAK60_6 motor3("Right_Slide", 3);
TMotorAK60_6 motor4("Right_Foot_Pitch", 51);
TMotorAK60_6 motor5("Right_Foot_Roll", 50);

TMotorAK80_80 motor6("Left_Roll", 11);
TMotorAK10_9 motor7("Left_Pitch", 10);
TMotorAK60_6 motor8("Left_Slide", 13);
TMotorAK60_6 motor9("Left_Foot_Pitch", 40);
TMotorAK60_6 motor10("Left_Foot_Roll", 41);

// Set up managers
MotorManager right_leg_motors("can0");
MotorManager left_leg_motors("can1");

float joint_1 = 0.0;
float joint_2 = 0.0;

// Joint state publisher
ros::Publisher leg_joint_publisher;

float joint_1 = 0.0;
float joint_2 = 0.0;

float inverse_kinematics_calc(float foot_pitch, float foot_roll, float Lx, float Ly, float Mx, float My, float Mz) {

    float alpha = -foot_pitch;
    float beta = foot_roll;
    float gamma = 0.0;
    float L = 192.37;
    float R = 40;

    float x = Lx*cos(beta);
    float y = Ly*cos(alpha) + Lx*sin(alpha)*sin(beta);
    float z = Ly*sin(alpha) - Lx*cos(alpha)*sin(beta);

    float h1 = L*L - (Mx-x)*(Mx-x);
    float i1 = R*R;
    float j1 = (z-Mz)/(My-y);
    float l1 = (-y*y + My*My + h1 - i1 + Mz*Mz - z*z)/ (2*My - 2*y);

    float A1 = j1*j1 + 1;
    float B1 = 2*(l1-y)*j1 - 2*z;
    float C1 = (l1-y)*(l1-y) + z*z - h1;

    float z_L = (-B1 - sqrt(B1*B1-4*A1*C1))/(2*A1);
    float y_L = z_L*j1 + l1;

    // Check if the values are real
    if (~isreal(z_L) || ~isreal(y1_L)){
        gamma = nanf("");
    }else {
        gamma = atan2((z1_L-Mz),(y1_L-My));
    }

    // Check if the output angle is out of range -90 < gamma < 90
    if (abs(gamma) > 90){
            gamma = nanf("");
    }

    return gamma;

}

// Calculates inverse kinematics for the ankle motors
void foot_inverse_kinematics(float &joint_1, float &joint_2, float foot_pitch, float foot_roll) {

    float alpha = -foot_pitch;
    float beta = foot_roll;

    float Lx = 52;
    float Ly = 48.94;
    float Mx = 53.5;
    float My = 60;
    float Mz = 185;

    joint_1 = inverse_kinematics_calc(alpha, beta, Lx, Ly, Mx, My, Mz);
    joint_2 = inverse_kinematics_calc(alpha, beta, -Lx, Ly, -Mx, My, Mz);
}

// Set postion goal subscriber
void setPositionGoal(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    std::vector<float> data = msg->data;
    

    // Log the data within msg to ROS_INFO
    //ROS_INFO("Setting torque goal, %f, measured torque %f", data[1], motor2.torque);
    
    // Left leg
    //motor1.send_position_goal(0.0);
    //motor2.send_position_goal(0.0);
    //motor3.send_position_goal(0.0);
    //motor4.send_position_goal(data[3]);
    //motor5.send_position_goal(data[4]);

    // Right leg
    //motor6.send_position_goal(0.0);
    //motor7.send_position_goal(0.0);
    //motor8.send_position_goal(0.0);

    pitch_rad = data[0] * M_PI / 180;
    roll_rad = data[1] * M_PI / 180

    float new_joint_1 = 0.0;
    float new_joint_2 = 0.0;

    foot_inverse_kinematics(new_joint_1, new_joint_2, pitch_rad, roll_rad);

    if(!isnan(new_joint_1) || !isnan(new_joint_2)){
        joint_1 = new_joint_1;
        joint_2 = new_joint_2;
    }

    ROS_INFO("Pitch: %f, Roll: %f", data[0], data[1]);
    ROS_INFO("joint_1: %f, joint_2: %f \n", joint_1 * M_PI / 180, joint_2 * M_PI / 180);

    motor9.send_position_goal(joint_1);
    motor10.send_position_goal(joint_2);

    // Update motor positions
    right_leg_motors.read_all();
    left_leg_motors.read_all();

    // Get joint states
    sensor_msgs::JointState leg_joint_states = right_leg_motors.get_joint_states();
    sensor_msgs::JointState right_leg_joint_states = left_leg_motors.get_joint_states();

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
//
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
    right_leg_motors.enable_all();
    left_leg_motors.enable_all();

    res.success = true;
    return true;
}

// Disable motors callback
bool disableMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Disable all motors");
    right_leg_motors.disable_all();
    left_leg_motors.disable_all();

    res.success = true;
    return true;
}

// Home motors callback
bool homeMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Homing all motors");
    right_leg_motors.home_all_individual(1.0);
    left_leg_motors.home_all_individual(1.0);
    
    res.success = true;
    return true;
}

// Main
int main(int argc, char **argv)
{
    // Adds all joints to the manager
    //right_leg_motors.add_motor(&motor1);
    //right_leg_motors.add_motor(&motor2);
    //right_leg_motors.add_motor(&motor3);
    //right_leg_motors.add_motor(&motor4);
    //right_leg_motors.add_motor(&motor5);

    left_leg_motors.add_motor(&motor6);
    left_leg_motors.add_motor(&motor7);
    left_leg_motors.add_motor(&motor8);
    left_leg_motors.add_motor(&motor9);
    left_leg_motors.add_motor(&motor10);

    // Print out motor names
    right_leg_motors.print_all_motors();
    left_leg_motors.print_all_motors();

    // Connect to motors and run them to 0.
    ROS_INFO("Connecting to right motors");
    right_leg_motors.connect();

    ROS_INFO("Enabling right motors");
    right_leg_motors.enable_all();

    ROS_INFO("Connecting to left motors");
    left_leg_motors.connect();
    left_leg_motors.enable_all();

    motor2.set_zero_offset(-0.15);
    motor8.set_zero_offset(1.0);
    motor8.set_position_limits(-2.0, 2.0);

    // motor1.run_to_home(0.5);
    // motor2.run_to_home(0.5);
    //motor3.run_to_home(5);
    //motor4.run_to_home(1);

    //motor6.run_to_home(0.5);
    //motor7.run_to_home(0.5);
    motor9.run_to_home(0.5);
    motor10.run_to_home(0.5);

    motor9.set_zero_offset(0.5);
    motor10.set_zero_offset(-0.5);

    motor9.run_to_home(0.5);
    motor10.run_to_home(0.5);

    // right_leg_motors.home_all_individual(0.5);


    // Set up left leg motor constants
    // motor1.set_constants(100.0, 4.0);
    // motor2.set_constants(100.0, 4.0);
    //motor3.set_constants(5.0, 0.5);
    //motor4.set_constants(20.0, 0.8);
    //motor5.set_constants(20.0, 0.8);

    //motor3.set_transmission_ratio(-0.013);

    // Set up right leg motor constants

    //motor6.copy_constants(&motor1);
    //motor6.set_constants(100.0, 0.2);
    //motor7.copy_constants(&motor2);
    //motor8.copy_constants(&motor3);
    //motor8.set_constants(5.0, 0.2);
    motor9.set_constants(10.0, 0.5);
    motor10.set_constants(10.0, 0.5);
    // motor10.set_constants(5.0, 1.0);

    //motor8.set_transmission_ratio(0.013);

    motor9.set_constants(10.0, 0.8);
    motor10.set_constants(10.0, 0.8);

    ROS_INFO("Starting subscriber node");

    // Init the node
    ros::init(argc, argv, "leg_motor_controller");

    ros::NodeHandle n;

    // Add services
    ros::ServiceServer enableService = n.advertiseService("/motors/enable_all", enableMotors);
    ros::ServiceServer disableService = n.advertiseService("/motors/disable_all", disableMotors);
    ros::ServiceServer homeService = n.advertiseService("/motors/home_all", homeMotors);

    ROS_INFO("Starting position goal");

    // Subscribe to the position goals topic
    ros::Subscriber sub = n.subscribe("/motors/position_goals", 1000, setPositionGoal);

    // Add publisher 
    leg_joint_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1000);

    // Spin the node
    ros::spin();

    return 0;
}
