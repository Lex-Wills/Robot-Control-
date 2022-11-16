
#include <iostream>
#include <boost/asio.hpp>
#include <thread>

#include "tcp/RobotServerComm.h"

#include "tcp/tcp_server.h"

#include "../rx150/RX150RobotInterface.h"

using namespace std;

/**
 * This class represents an entirely virtual RX150 robot arm.
 * It merely manages four join angles that can be read and manipulated through setting target joint angles. 
 */
class SolutionRX150SimulatedInterface : public RX150RobotInterface {
public:
    static std::shared_ptr<RX150RobotInterface> create(){
        return std::shared_ptr<RX150RobotInterface>(new SolutionRX150SimulatedInterface());
    }

    SolutionRX150SimulatedInterface(){
	// initialize all data members
        currentJointAngles = Eigen::VectorXd(numberOfJoints());
        currentJointAngles << 0,0,0,0,0; // initial joint angles in radiant
        hasTargetJointAngles = false;
        targetJointAngles = Eigen::VectorXd();
        jointGain = 0.1;
    }

    Eigen::VectorXd getCurrentJointAngles(){
	// simply return stored value
        return currentJointAngles;
    }

    void setTargetJointAngles(Eigen::VectorXd targets){
        if(targets.rows()==0){
	    // unset target
            this->hasTargetJointAngles = false;
            return;
        }
        // check for correct dimension
        else if(targets.rows()!=numberOfJoints()){
            cerr << "Invalid number of joint angles: " << targets.rows() << endl;
        }
        else{
            this->hasTargetJointAngles = true;
            Eigen::VectorXd newTargets = targets;
            // clip angles to legal joint range
            newTargets = newTargets.cwiseMax(getJointLowerLimits());
            newTargets = newTargets.cwiseMin(getJointUpperLimits());
            this->targetJointAngles = targets;
        }
    }

    Eigen::VectorXd getTargetJointAngles(){
        if(hasTargetJointAngles) {
            return targetJointAngles;
        }
        else{
            return Eigen::VectorXd();
        }
    }

    void step(){
        if(hasTargetJointAngles){
            // move a step towards target joint angles
            currentJointAngles += jointGain * (targetJointAngles-currentJointAngles);
        }
        // else: don't move
    }
private:
    Eigen::VectorXd currentJointAngles;
    bool hasTargetJointAngles;
    Eigen::VectorXd targetJointAngles;
    double jointGain;
};


int main(int argc, char* argv[]){
    // IMPORTANT: DO  NOT  TOUCH  THIS  METHOD
    try{
	// open network port
        boost::asio::io_service io_service;
        auto robot = SolutionRX150SimulatedInterface::create();
        tcp_server server(io_service, ROBOT_SERVER_PORT, robot);
	
	// in parallel: call step() function periodically 
        thread modelUpdater([robot](){
            while(true){
                ((SolutionRX150SimulatedInterface*)robot.get())->step();
                usleep(10*1000);
            }
        });

	// tcp_server will now delegant all network requests to methods of RX150SimulatedInterface
        io_service.run();
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    return 0;
}
