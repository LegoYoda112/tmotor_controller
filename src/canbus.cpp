#include "tmotor_controller/canbus.h"

// Connect to a can bus
int canbus::connect(){
    // Create a CAN socket
    int s;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return 1;
    }

    // Connect to that socket
    struct ifreq ifr;

    strcpy(ifr.ifr_name, name.c_str());
    ioctl(s, SIOCGIFINDEX, &ifr);

    std::cout << "Connecting to " << ifr.ifr_name << std::endl;

    struct sockaddr_can addr;

    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if( bind(s, (struct sockaddr *) &addr, sizeof(addr)) < 0){
        perror("Bind");
        return 1;
    }   

    std::cout << "socket: " << s << std::endl;

    this->socket_num = s;
    return s;
}

void canbus::disconnect(){
    printf("Closing socket\n");

    // Closes the socket
    int s = this->socket_num;
    if (close( s ) < 0) {
        perror("Close");
    }   
}

// Return the socket number
int canbus::get_socket_num(){
    return this->socket_num;
}

std::string canbus::get_name(){
    return this->name;
}

// Reads all the data out of a socket's buffer
void canbus::clear_buffer(){
    int bytes_left;
    int nbytes;
    int size = 100;
    struct can_frame frame;

    read(this->get_socket_num(), &frame, sizeof(struct can_frame));
    read(this->get_socket_num(), &frame, sizeof(struct can_frame));
    read(this->get_socket_num(), &frame, sizeof(struct can_frame));
    read(this->get_socket_num(), &frame, sizeof(struct can_frame));


    std::cout << "Stopped reading" << std::endl;
}