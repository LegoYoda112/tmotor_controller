#include "TMotor.h"

#include "canbus.h"

#include <string.h>
#include <vector>

#ifndef MOTORMANAGER_H
#define MOTORMANAGER_H

class MotorManager
{
    protected:
        int socket_num;
        canbus bus;
        vector<TMotor*> motors;
    
    public:
        MotorManager(std::string name){
            this->bus = canbus(name);
        }

        void connect();

        void add_motor(TMotor* motor);

        void print_all_motors();

        void enable_all();

        void disable_all();

        void home_all();

        void read_all();

        void home_all_individual(float);

        void soft_stop_hold();

        void soft_stop_dampen();
};

#endif