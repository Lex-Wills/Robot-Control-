
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <../rx150/RX150Kinematics.h>
#include <tcp/RX150RemoteInterface.h>

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]){

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }
    shared_ptr<RX150RobotInterface> robot = RX150RemoteInterface::create(remoteHost);

    uint t = 0;
    while(true){
        cout << "Enter command: " << endl;
        string input;
        getline(cin,input);
        stringstream inputS(input);
        string command;
        inputS >> command;

        if(command == "set" || command == "s"){
            VectorXd angles(5);
            bool fail = false;
            for(uint i=0;i<5;i++){
                double t;
                inputS >> t;
//                cout << t << endl;
                if(inputS.fail()){
                    fail = true;
                    break;
                }
                angles[i] = t;
            }
            if(fail){
                cerr<< "Invalid joint angles." << endl;
            }
            else{
                robot->setTargetJointAngles(angles);
            }
        }
        else if(command == "zero" || command == "z"){
            VectorXd angles = VectorXd::Zero(5);
            robot->setTargetJointAngles(angles);
        }
        else if(command == "inc" || command == "i"){
            VectorXd angles = robot->getCurrentJointAngles();
            bool fail = false;
            int idx;
            inputS >> idx;
            if(inputS.fail() || idx < 1 || 5 < idx){
                fail = true;
                cerr<< "Invalid joint id [1...5]." << endl;
            }
            double t;
            inputS >> t;
            if(inputS.fail()){
                fail = true;
                cerr<< "Invalid joint angle." << endl;
            }
            if(!fail){
                angles[idx-1]+=t;
                robot->setTargetJointAngles(angles);
            }
        }
        else if(command == "print" || command == "p"){
            cout << robot->getCurrentJointAngles() << endl;
        }
        else if(command == "help" || command == "h"){
            cout << "Available commands: " << endl;
            cout << "\thelp/h: show this message" << endl;
            cout << "\tprint/p: print current joint angles" << endl;
            cout << "\tset/s angle1 angle2 angle3 angle4 angle5: set target joint angles" << endl;
            cout << "\tinc/i joint angle: increment specific joint by an angle" << endl;
            cout << "\tzero/z: set joint angles to zero" << endl;
        }
        else{
            cerr << endl << "Unknown command: \"" << command << "\"" << endl;
        }

//        int jointID;
//        double increment;
//
//	    // get current joint angles from robot
//	    VectorXd currentJoint = robot->getCurrentJointAngles();
//
//	    // TODO determine current effector position from currentJoint
//        // (implement and use AX18AKinematics::computeForwardKinematics)
//
//        // TODO write joint angles to robot
//
//        robot->setTargetJointAngles(Eigen::VectorXd::Ones(5));
//
//        usleep(5000*1000);
//        robot->setTargetJointAngles(Eigen::VectorXd::Zero(5));
//
//        usleep(5000*1000);
//
//        usleep(25*1000);
    }

    return 0;
}
