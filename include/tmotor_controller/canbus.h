#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#ifndef CAN_H
#define CAN_H

class canbus
{
    protected:
        int socket_num;
        std::string name;

    public:

        canbus(){
            this->name = "can0";
        }

        canbus(std::string name){
            this->name = name;
        }

        int connect();

        void disconnect();

        int get_socket_num();

        std::string get_name();

        void clear_buffer();

        int get_waiting_bytes();
};

#endif