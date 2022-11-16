//
// Created by matthias on 10/03/18.
//

#ifndef ROBOT_SERVER_SERVERCLIENT_COMM_H_H
#define ROBOT_SERVER_SERVERCLIENT_COMM_H_H


#include <boost/asio.hpp>

#include <Eigen/Eigen>

#define ROBOT_SERVER_PORT 10000
#define ROBOT_SERVER_PORT_STR "10000"

#define ROBOT_SERVER_ECHO_REQUEST ((uint32_t)(1<<31))

#define ROBOT_SERVER_GET_JOINT_ANGLES (uint32_t)1

#define ROBOT_SERVER_SET_JOINT_COMMAND ((uint32_t)(1<<16))



void writeCommandToSocket(boost::asio::ip::tcp::socket &socket, uint32_t command);

void checkCommandEchoFromSocket(boost::asio::ip::tcp::socket &socket, uint32_t command);

Eigen::VectorXd readVectorXdFromSocket(boost::asio::ip::tcp::socket &socket, size_t expectedSize);

void writeVectorXdToSocket(boost::asio::ip::tcp::socket &socket, Eigen::VectorXd vector);


#endif //ROBOT_SERVER_SERVERCLIENT_AX_18A_COMM_H_H
