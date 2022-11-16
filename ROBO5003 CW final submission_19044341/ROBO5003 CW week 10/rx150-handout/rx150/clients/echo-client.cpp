
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <tcp/RX150RemoteInterface.h>

#include "tcp/RobotServerComm.h"

using namespace std;
using namespace Eigen;

int main(int argc, char* argv[]){

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }

    shared_ptr<RX150RobotInterface> robot = RX150RemoteInterface::create(remoteHost);

    while(true){
        VectorXd joints = robot->getCurrentJointAngles();

        cout << "Joint angles received: " << joints.transpose() << endl;

        usleep(250*1000);

    }

    return 0;
}
