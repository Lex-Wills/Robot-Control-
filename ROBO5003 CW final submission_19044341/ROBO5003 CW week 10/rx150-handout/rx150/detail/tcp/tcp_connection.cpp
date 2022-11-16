//
// Created by matthias on 11/03/18.
//

#include "tcp_connection.h"

#include <boost/array.hpp>

#include <iostream>

#include "RobotServerComm.h"

using boost::asio::ip::tcp;
using namespace std;

tcp_connection::pointer tcp_connection::create(boost::asio::io_context& io_context,std::shared_ptr<RX150RobotInterface> robotP){
    return pointer(new tcp_connection(io_context, robotP));
}

tcp::socket& tcp_connection::socket(){
    return socket_;
}

void tcp_connection::start(){
    this->handleClient();
}

void tcp_connection::handleClient(){
    boost::system::error_code error;

    // FIXED if this is not set, buffering algorithm causes long delays
    boost::asio::ip::tcp::no_delay option(true);
    socket_.set_option(option);

    while(true){

        boost::array<uint32_t, 1> command;
        size_t bytesLength = socket_.read_some(boost::asio::buffer(command), error);
        //cout << "received " << bytesLength << " bytes." << endl;

        if (error == boost::asio::error::eof)
            break; // Connection closed cleanly by peer.
        else if (error)
            throw boost::system::system_error(error); // Some other error.

        if(command[0] == ROBOT_SERVER_GET_JOINT_ANGLES){

            writeCommandToSocket(socket_, ROBOT_SERVER_ECHO_REQUEST | ROBOT_SERVER_GET_JOINT_ANGLES);

            writeVectorXdToSocket(socket_, robot->getCurrentJointAngles());

        }
        else if(command[0] == ROBOT_SERVER_SET_JOINT_COMMAND){
            writeCommandToSocket(socket_, ROBOT_SERVER_ECHO_REQUEST | ROBOT_SERVER_SET_JOINT_COMMAND);

            Eigen::VectorXd command = readVectorXdFromSocket(socket_, robot->numberOfJoints());

            robot->setTargetJointAngles(command);
        }
        else{
            cerr << "Unknown command received from client: " << command[0] << endl;
            abort();
        }
    }
}
tcp_connection::tcp_connection(boost::asio::io_context& io_context, std::shared_ptr<RX150RobotInterface> robotP)
: socket_(io_context), robot(robotP)
        {
        }
