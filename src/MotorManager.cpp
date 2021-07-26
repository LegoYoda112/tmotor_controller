#include "tmotor_controller/MotorManager.h"

// Connect to canbus
void MotorManager::connect(){
    this->bus.connect();
}

// Appends motor to motor list
void MotorManager::add_motor( TMotor *motor ) {
    this->motors.push_back(motor);
}

// Print out the list of motors
void MotorManager::print_all_motors(){
    cout << "=== MOTORS CONNECTED ON \"" << this->bus.get_name() << "\" ==="<< endl;

    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;

        cout << "Joint name: " << motor->joint_name;
        cout << ", Motor type: " << motor->motor_type;
        cout << ", CAN ID: " << (int) motor->can_id;
        cout << endl;
    }

    cout << endl;
}

// Enables all motors in the manager
void MotorManager::enable_all(){
    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;

        // Sets CAN socket on all motors
        motor->set_socket(this->bus.get_socket_num());

        // Sends disable
        motor->send_disable();

        // Sends enable command
        motor->send_enable();

        // Waits for motor to reply
        motor->read_motor_response();

        // Sends 0 goal to overwrite any existing commands
        motor->send_velocity_goal(0.0);

        // Reads motor response
        motor->read_motor_response();
    }
}

// Disables all motors in the manager
void MotorManager::disable_all(){
    cout << "Sending Disable" << endl;

    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;

        // Sends enable command
        motor->send_disable();
    }
}

// Read the state of all the motors
void MotorManager::read_all(){

    // Get number of motors
    int num_motors = this->motors.size();

    // Loop through all motors
    for(int i = 0; i < num_motors; i++){

        int nbytes;
        struct can_frame frame;
        nbytes = read(bus.get_socket_num(), &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            perror("Read");
        }

        // Grab the CAN frame ID
        int id = frame.data[0];

        // Loop through all motors and set correct based on CAN ID
        for (auto it = this->motors.begin(); it != this->motors.end(); it++){
            auto motor = *it;

            if(id == motor->can_id){
                motor->read_motor_response_from_frame(frame);
            }
        }
    }
}

// Homes each motor one by one
void MotorManager::home_all_individual(float speed){
    // Loop through all motors and set correct based on can id
    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;

        motor->run_to_home(speed);
    }
}

// TODO: make all motors stop and weakly hold their position
void MotorManager::soft_stop_hold(){

    // Loop through motor vector and set position goals to current position
    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;
        motor->send_position_goal(motor->position);
    }
}

// TODO: make all motors come to a stop
void MotorManager::soft_stop_dampen(){
    // Loop through motor vector and set velocity goals to 0
    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;
        motor->send_velocity_goal(0.0);
    }
}

// Returns ROS JointState messages for all motors
sensor_msgs::JointState MotorManager::get_joint_states(){

    sensor_msgs::JointState joint_state;

    // Loop through motor vector and get states
    for (auto it = this->motors.begin(); it != this->motors.end(); it++){
        auto motor = *it;

        joint_state.name.push_back(motor->joint_name);
        joint_state.position.push_back(motor->position * motor->get_transmission_ratio());
        joint_state.velocity.push_back(motor->velocity * motor->get_transmission_ratio());
        joint_state.effort.push_back(motor->torque / motor->get_transmission_ratio());

    }

    return joint_state;

}