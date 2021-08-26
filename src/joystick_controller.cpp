#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "std_srvs/Trigger.h"

#include "std_msgs/Float32MultiArray.h"

float joint_1 = 0.0;
float joint_2 = 0.0;
float joint_3 = 0.0;
float joint_4 = 0.0;
float joint_5 = 0.0;

float enable = 0.0;

ros::ServiceClient disableMotors;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

  // joint_1 = joy->axes[0];

  joint_1 = joy->axes[3] / 2.0;
  joint_2 = joy->axes[0] / 2.0;
  joint_3 = joy->axes[1] / 2.0;

  joint_4 = (joy->axes[3] / 2.0 + joy->axes[4] ) / 2.0;
  joint_5 = (joy->axes[3] / 2.0 - joy->axes[4] ) / 2.0;

  if(joy->buttons[1] == 1){
      ROS_INFO("STOP");

      std_srvs::Trigger disableMotorsTrigger;

      disableMotors.call(disableMotorsTrigger);

      joint_1 = 0;
      joint_2 = 0;
      joint_3 = 0;
  }    
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joystick_controller");

  ros::NodeHandle n;

  ros::Publisher position_pub = n.advertise<std_msgs::Float32MultiArray>("/motors/position_goals", 1000);

  ros::Rate loop_rate(8);

  // ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);

  disableMotors = n.serviceClient<std_srvs::Trigger>("/motors/disable_all");

  int length = 61 * 61;
  float angles[length][2];

  int i = 0;

  for(int pitch = -20; pitch <= 20; pitch++){
    if(pitch % 2  == 0) {
      for(int roll = -20; roll <= 20; roll++){
       angles[i][0] = pitch;
       angles[i][1] = roll;
       i++;
     }
    }else{
      for(int roll = 20; roll >= -20; roll--){
       angles[i][0] = pitch;
       angles[i][1] = roll;
       i++;
     }
    }
  }

  int count = 0;
  i = 0;
  while (ros::ok())
  {
    std_msgs::Float32MultiArray msg;

    float pitch = angles[i][0];
    float roll = angles[i][1];
    i++;

    std::cout << "Commanded pitch: " << pitch << " commanded roll: " << roll << std::endl;

    msg.data = {pitch, roll, joint_3, joint_4, joint_5};

    position_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
