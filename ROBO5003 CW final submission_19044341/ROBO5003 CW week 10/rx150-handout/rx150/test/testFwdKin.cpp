
#define BOOST_TEST_COLOR_OUTPUT 1
#define BOOST_TEST_MODULE FwdKinTest
#include <boost/test/unit_test.hpp>



// boost has non-virtual constructors ^^
#pragma GCC diagnostic ignored "-Weffc++"

#include <../rx150/RX150Kinematics.h>

#include "testUtilities.h"

#include <iostream>
#include <sstream>
#include <armadillo>

using namespace Eigen;
using namespace std;


BOOST_AUTO_TEST_SUITE(ForwardKinematics)

BOOST_AUTO_TEST_CASE(NoThrow){
	Eigen::MatrixXd mat = RX150Kinematics::computeForwardKinematics(Eigen::VectorXd(5));
}

BOOST_AUTO_TEST_CASE(Throw){
	BOOST_CHECK_THROW(RX150Kinematics::computeForwardKinematics(Eigen::VectorXd()),std::invalid_argument);
	BOOST_CHECK_THROW(RX150Kinematics::computeForwardKinematics(Eigen::VectorXd(4)),std::invalid_argument);
	BOOST_CHECK_THROW(RX150Kinematics::computeForwardKinematics(Eigen::VectorXd(6)),std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(Initial_Pos){
    Eigen::VectorXd joints(5);
    joints << 0,0,0,1,0;
    Eigen::MatrixXd mat = RX150Kinematics::computeForwardKinematics(joints);
    cout << mat;
}

BOOST_AUTO_TEST_CASE(FwdKin_Size){
    Eigen::MatrixXd mat = RX150Kinematics::computeForwardKinematics(Eigen::VectorXd(5));
    BOOST_CHECK_EQUAL(mat.rows(), 4);
    BOOST_CHECK_EQUAL(mat.cols(), 4);
}

BOOST_AUTO_TEST_CASE(Scale){
        Eigen::VectorXd joints(5);
        joints << 0,0,0,0,0;
        for (int i=0; i < 5; i++){
            joints[i] = 1;
            Eigen::MatrixXd mat = RX150Kinematics::computeForwardKinematics(joints);
            BOOST_CHECK_EQUAL(mat(3,0), 0);
            BOOST_CHECK_EQUAL(mat(3,1), 0);
            BOOST_CHECK_EQUAL(mat(3,2), 0);
            BOOST_CHECK_EQUAL(mat(3,3), 1);
        }
}

BOOST_AUTO_TEST_SUITE_END()

