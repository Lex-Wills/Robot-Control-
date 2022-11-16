//
// Created by matthias on 11/03/18.
//

#include "RX150RemoteInterface.h"
#include "RobotServerComm.h"

#include <iostream>
#include <boost/array.hpp>
#include <chrono>	  // timer

using namespace std;
using boost::asio::ip::tcp;


std::shared_ptr<RX150RobotInterface> RX150RemoteInterface::create(string host){
    return std::shared_ptr<RX150RobotInterface>(new RX150RemoteInterface(host));
}

RX150RemoteInterface::RX150RemoteInterface(string host){
    boost::asio::io_service io_service;

    socket = new tcp::socket(io_service);

    tcp::resolver resolver(io_service);
    boost::asio::connect(*socket, resolver.resolve({host, ROBOT_SERVER_PORT_STR}));

    // FIXED if this is not set, buffering algorithm causes long delays
    boost::asio::ip::tcp::no_delay option(true);
    socket->set_option(option);

    currentJointAngles = Eigen::VectorXd(4);

    initialDataRead = false;

    running = true;
    updateMicros = 40*1000;
    //updateMicros = 1000*1000;
    //updateMicros = 10;
    this->updater = thread(&RX150RemoteInterface::runUpdates, this);

}

RX150RemoteInterface::~RX150RemoteInterface(){
    delete socket;
    running = false;
}

Eigen::VectorXd RX150RemoteInterface::getCurrentJointAngles(){
    Eigen::VectorXd joints;
    {
        unique_lock<mutex> lk(dataMutex);
        if(initialDataRead){
            joints = this->currentJointAngles;
        }
        else{
            cout << "Waiting for initial data." << endl;
            initialDataReadNotifier.wait(lk);
            cout << "Initial data received." << endl;
            joints = this->currentJointAngles;
            initialDataRead = true;
        }

    }

    return joints;
}

void RX150RemoteInterface::setTargetJointAngles(Eigen::VectorXd targets){
    if(targets.rows()!=numberOfJoints()){
        throw invalid_argument("RX150RemoteInterface::setTargetJointAngles(): Wrong number of target joint angles");
    }

    unique_lock<mutex> lk(dataMutex);
    this->targetJointAngles = targets;
}

Eigen::VectorXd RX150RemoteInterface::getTargetJointAngles(){
    Eigen::VectorXd t;
    {
        unique_lock<mutex> lk(dataMutex);
        t = this->targetJointAngles;
    }
    return t;
}

Eigen::VectorXd RX150RemoteInterface::getCurrentJointAnglesFromServer(){
    boost::system::error_code error;

    writeCommandToSocket(*socket, ROBOT_SERVER_GET_JOINT_ANGLES);

    checkCommandEchoFromSocket(*socket, ROBOT_SERVER_GET_JOINT_ANGLES);

    Eigen::VectorXd anglesEigen = readVectorXdFromSocket(*socket, numberOfJoints());

    return anglesEigen;
}

void RX150RemoteInterface::writeTargetJointAnglesToServer(Eigen::VectorXd angles){

    assert(angles.rows()==numberOfJoints());

    writeCommandToSocket(*socket, ROBOT_SERVER_SET_JOINT_COMMAND);
    checkCommandEchoFromSocket(*socket, ROBOT_SERVER_SET_JOINT_COMMAND);
    writeVectorXdToSocket(*socket, angles);

}


void RX150RemoteInterface::runUpdates(){
    while(running){
        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::VectorXd serverCurrentAngles = getCurrentJointAnglesFromServer();
        //Eigen::VectorXd serverCurrentAngles;

        Eigen::VectorXd command;

        dataMutex.lock();
        this->currentJointAngles = serverCurrentAngles;
        initialDataReadNotifier.notify_all();
        command = this->targetJointAngles;
        dataMutex.unlock();

        if(command.rows() > 0){
            writeTargetJointAnglesToServer(command);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();

        if(micros < updateMicros){
            usleep( updateMicros - micros  );
        }
        else{
            cerr << "Warning: Server update takes too long: " << micros << "microseconds." << endl;
        }
    }
}
