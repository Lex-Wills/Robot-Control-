
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

    robot->setTargetJointAngles(Eigen::VectorXd::Zero(5));
    usleep(2000*1000);

    uint t = 0;
    VectorXd currentJointAngles = robot->getCurrentJointAngles();
    while(true){

        // alternate between  target coordinates
        // TODO choose a good set of targets to allow for effective testing
        VectorXd targetPosition(3);
        switch((t++/100)%2){
            case 0:
                targetPosition << 0.0, 1.0, 0.0;
                break;
            case 1:
                targetPosition << 0.0, 0.0, 1.0;
                break;

            case 2:
                targetPosition << 0.111578, -1.6251e-17, -0.0242984;
                break;
            case 3:
                targetPosition << 0.240595, -9.10723e-18, 0.0923676;
                break;
            case 4:
                targetPosition << 0.199971, 0.113943, -0.0241301;
                break;
        }

        // TODO compute next motor command
        int k = 0.0001;

        Eigen::MatrixXd fwd_kin = RX150Kinematics::computeForwardKinematics(currentJointAngles);

        Eigen::VectorXd pos_vector = Eigen::VectorXd::Zero(3);

        for (int i=0; i < 3; i++) {
            pos_vector(i) = fwd_kin(i, 3);
        }

        Eigen::MatrixXd J = RX150Kinematics::computeJacobianMatrix(currentJointAngles);

        Eigen::VectorXd desired_motion = k * (targetPosition - pos_vector);

        Eigen::MatrixXd j_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        Eigen::MatrixXd targetJointAngles = Eigen::VectorXd::Zero(5);

        for (int i=0; i < 5; i++) {
            Eigen::VectorXd j_row = Eigen::VectorXd::Zero(3);
            for (int j; j < 3; j++){
                j_row(j) = j_inverse(i, j);
            }
            targetJointAngles(i) = j_row.dot(desired_motion);
        }

        Eigen::MatrixXd newJointAngles = currentJointAngles.array() + targetJointAngles.array();

        //TODO compute nullspace contol
        int k_null = 0.05;

        Eigen::VectorXd home_posture(5);
        home_posture << 0, 0.589041, 1.11918, -0.589041, 0.647945;
        Eigen::VectorXd delta_home = k_null * (home_posture - currentJointAngles);

        Eigen::VectorXd j_tmp_vec = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd j_inv_tmp_vec = Eigen::VectorXd::Zero(6);
        Eigen::MatrixXd j_null = Eigen::MatrixXd::Zero(5,5);

        for (int i = 0; i < 5; i++) {
            std::cout << j_inverse(i, Eigen::all);
            std::cout << "\n\n";
            j_inv_tmp_vec << j_inverse(i, Eigen::all);
            for (int j = 0; j < 5 ; j++) {
                j_tmp_vec << J(Eigen::all, j);
                j_null(i, j) = j_inv_tmp_vec.dot(j_tmp_vec);
            }
        }

        Eigen::MatrixXd j_null_i = MatrixXd::Identity(5,5) - j_null;
        Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(5);

        for (int n = 0; n < 5; n++) {


            tmp_vec << j_null_i(n, Eigen::all);
            newJointAngles(n) += tmp_vec.dot(delta_home);
        }

        robot->setTargetJointAngles(newJointAngles);

        std::cout << newJointAngles;
        std::cout << pos_vector;
        std::cout << "\n\n";

        usleep(50*1000);
    }

    return 0;
}



