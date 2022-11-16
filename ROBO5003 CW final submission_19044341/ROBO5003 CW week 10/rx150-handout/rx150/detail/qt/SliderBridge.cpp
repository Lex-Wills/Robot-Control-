
#include "SliderBridge.h"

#include <iostream>


SliderBridge::SliderBridge(std::shared_ptr<RX150RobotInterface> robotP)
 : robot(robotP), q(Eigen::VectorXd::Ones(5)*50) {}

void SliderBridge::setOne(double value){
	q[0] = value;
	send();
}
void SliderBridge::setTwo(double value){
	q[1] = value;
	send();
}
void SliderBridge::setThree(double value){
	q[2] = value;
	send();
}
void SliderBridge::setFour(double value){
	q[3] = value;
	send();
}
void SliderBridge::setFive(double value){
	q[4] = value;
	send();
}
    

void SliderBridge::send(){
	Eigen::VectorXd qNorm = (q-Eigen::VectorXd::Ones(5)*50)*(1.0/50*3.14);
	std::cout << qNorm << std::endl;
	robot->setTargetJointAngles( qNorm );
}



