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
TMotorAK80_80 right_roll("Right_Roll", 1);
TMotorAK10_9 right_pitch("Right_Pitch", 2);
TMotorAK60_6 right_slide("Right_Slide", 3);
TMotorAK60_6 right_inner_ankle("Right_Foot_Pitch", 50);
TMotorAK60_6 right_outer_ankle("Right_Foot_Roll", 51);

TMotorAK80_80 left_roll("Left_Roll", 11);
TMotorAK10_9 left_pitch("Left_Pitch", 10);
TMotorAK60_6 left_slide("Left_Slide", 13);
TMotorAK60_6 left_inner_ankle("Left_Foot_Pitch", 40);
TMotorAK60_6 left_outer_ankle("Left_Foot_Roll", 41);

// Set up managers
MotorManager right_leg_motors("can0");
MotorManager left_leg_motors("can1");

float joint_1 = 0.0;
float joint_2 = 0.0;

// Joint state publisher
ros::Publisher leg_joint_publisher;

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
    if (isnan(z_L) || isnan(y_L)){
        gamma = nanf("");
    }else {
        gamma = atan2((z_L-Mz),(y_L-My));
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
    
    // Right leg
    right_roll.send_position_goal(0.0);
    right_pitch.send_position_goal(0.0);
    right_slide.send_position_goal(0.0);
    right_inner_ankle.send_position_goal(0.0);
    right_outer_ankle.send_position_goal(0.0);

    // Left leg
    left_roll.send_position_goal(0.0);
    left_pitch.send_position_goal(0.0);
    left_slide.send_position_goal(0.0);

    float pitch_rad = data[0] * M_PI / 180;
    float roll_rad = data[1] * M_PI / 180;

    float new_joint_1 = 0.0;
    float new_joint_2 = 0.0;

    foot_inverse_kinematics(new_joint_1, new_joint_2, pitch_rad, roll_rad);

    // Safety catch to make sure we don't ask the joints
    // to go to "nan"
    if(!isnan(new_joint_1) || !isnan(new_joint_2)){
        joint_1 = new_joint_1;
        joint_2 = new_joint_2;
    }

    ROS_INFO("Pitch: %f, Roll: %f", data[0], data[1]);
    ROS_INFO("joint_1: %f, joint_2: %f \n", joint_1 * M_PI / 180, joint_2 * M_PI / 180);

    // Send ankle joint goals
    left_inner_ankle.send_position_goal(joint_1);
    left_outer_ankle.send_position_goal(joint_2);

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

// ================== Enable motors callback
bool enableMotors(std_srvs::Trigger::Request& req,
                  std_srvs::Trigger::Response& res){
    ROS_INFO("Enable all motors");
    right_leg_motors.enable_all();
    left_leg_motors.enable_all();

    res.success = true;
    return true;
}

// ================== Disable motors callback
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
    right_leg_motors.add_motor(&right_roll);
    right_leg_motors.add_motor(&right_pitch);
    right_leg_motors.add_motor(&right_slide);
    right_leg_motors.add_motor(&right_inner_ankle);
    right_leg_motors.add_motor(&right_outer_ankle);

    left_leg_motors.add_motor(&left_roll);
    left_leg_motors.add_motor(&left_pitch);
    left_leg_motors.add_motor(&left_slide);
    left_leg_motors.add_motor(&left_inner_ankle);
    left_leg_motors.add_motor(&left_outer_ankle);

    // Print out motor names
    right_leg_motors.print_all_motors();
    left_leg_motors.print_all_motors();


    // ====================== CONNECT AND ENABLE MOTORS ======================
    // Connect to motors and run them to 0.
    ROS_INFO("Connecting to right motors");
    right_leg_motors.connect();
    ROS_INFO("Enabling right motors");
    right_leg_motors.enable_all();

    ROS_INFO("Connecting to left motors");
    left_leg_motors.connect();
    ROS_INFO("Enabling right motors");
    left_leg_motors.enable_all();


    // ====================== SET OFFSETS AND LIMITS ======================
    ROS_INFO("Setting up zero offsets and limits");

    // Right Leg
    right_pitch.set_zero_offset(-0.15);

    // Left Leg
    left_slide.set_zero_offset(1.0);
    left_slide.set_position_limits(-2.0, 2.0);

    //motor9.set_zero_offset(0.5);
    //motor10.set_zero_offset(-0.5);

    // ====================== HOME MOTORS ======================
    ROS_INFO("Homing all motors");

    // Right Leg
    right_roll.run_to_home(0.1);
    right_pitch.run_to_home(0.1);
    right_slide.run_to_home(5);
    right_inner_ankle.run_to_home(0.5);
    right_outer_ankle.run_to_home(0.5);
    

    // Left Leg
    left_roll.run_to_home(0.1);
    left_pitch.run_to_home(0.1);
    left_slide.run_to_home(5);
    left_inner_ankle.run_to_home(0.5);
    left_outer_ankle.run_to_home(0.5);

    // ====================== SET MOTOR CONSTANTS ======================
    ROS_INFO("Setting motor constants");

    right_roll.set_constants(100.0, 4.0); // Roll
    right_pitch.set_constants(100.0, 4.0); // Pitch
    right_slide.set_constants(10.0, 0.5); // Slide
    right_inner_ankle.set_constants(10.0, 0.5);
    right_outer_ankle.set_constants(10.0, 0.5);

    // Left Leg
    left_roll.copy_constants(&right_roll); // Duplicate equivelent motor constants
    left_pitch.copy_constants(&right_pitch);
    left_slide.copy_constants(&right_slide);
    left_inner_ankle.set_constants(10.0, 0.5);
    left_outer_ankle.set_constants(10.0, 0.5);

    // Send zero position goal to hold position
    right_leg_motors.send_all_zero();
    left_leg_motors.send_all_zero();


    // ====================== START NODE ======================
    ROS_INFO("Starting node");

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
