#include <iostream>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <math.h>
#include <cmath>

#ifndef TMOTOR_H
#define TMOTOR_H

using namespace std;

class TMotor
{
    protected:
        // Motor constants
        const float P_MIN = -12.5;
        const float P_MAX = 12.5;
        const float V_MIN = -41.87;
        const float V_MAX = 41.87;
        const float T_MIN = -9;
        const float T_MAX = 9;
        const float KP_MIN = 0;
        const float KP_MAX = 500;
        const float KD_MIN = 0;
        const float KD_MAX = 5;

        // Control constants
        float p_des = 0.0;
        float v_des = 0.0;
        float kP = 0.0;
        float kD = 0.0;
        float t_ff = 0.0;

        // Other physical constants
        float transmission_ratio = 1.0;
        float zero_offset = 0.0;

    public:
        string joint_name = "default";
        string motor_type;

        uint8_t can_id = 0;
        int socket = 0;

        float position;
        float velocity;
        float torque;

        // Default constructor
        TMotor(){

        }

        TMotor(string joint_name){
            this->joint_name = joint_name;
        }

        // Pack and unpack functions
        void pack_motor_cmd(uint8_t*, float, float, float, float, float);

        void unpack_motor_response(uint8_t*, float&, float&, float&);

        // Set functions
        void set_constants(float, float);

        void set_socket(int);

        void set_zero_offset(float);

        void set_transmission_ratio(float);

        // Get functions
        float get_transmission_ratio();

        // Send functions
        void send_can_msg(uint8_t*);

        void send_motor_cmd(float, float, float, float, float);

        void send_enable();

        void send_disable();

        void send_zero_encoder();

        void send_position_goal(float, float = 0.0);

        void send_velocity_goal(float, float = 0.0);

        void send_torque_goal(float);

        void read_motor_response();

        void read_motor_response_from_frame(struct can_frame frame);

        // Routine functions
        void run_to_home(float seconds);
        
};


#endif