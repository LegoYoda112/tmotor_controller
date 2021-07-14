// Header file with specific constants for the AK80_80 motor

#include <string>

#include "TMotor.h"

class TMotorAK80_80 : public TMotor
{
    protected:
        // Motor constants
        const float P_MIN = -12.5;
        const float P_MAX = 12.5;
        const float V_MIN = -2.93;
        const float V_MAX = 2.93;
        const float T_MIN = -144;
        const float T_MAX = 144;
        const float KP_MIN = 0;
        const float KP_MAX = 500;
        const float KD_MIN = 0;
        const float KD_MAX = 5;

    public:
        TMotorAK80_80(string joint_name, int can_id){
            
            // Can socket
            this->socket = socket;

            this->joint_name = joint_name;

            this->motor_type = "TMotorAK80_80";

            this->can_id = can_id;
        }
};
