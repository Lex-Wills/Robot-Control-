
#define BOOST_TEST_COLOR_OUTPUT 1
#define BOOST_TEST_MODULE CTFromDHTest
#include <boost/test/unit_test.hpp>


// boost has non-virtual constructors ^^
#pragma GCC diagnostic ignored "-Weffc++"

#include <../rx150/RX150Kinematics.h>

#include "testUtilities.h"

#include <iostream>
#include <sstream>

using namespace Eigen;
using namespace std;




BOOST_AUTO_TEST_SUITE(DHTransformation)

BOOST_AUTO_TEST_CASE(NoThrow){
	Eigen::MatrixXd mat = RX150Kinematics::getTransformMatrixFromDH(0,0,0,0);
}

BOOST_AUTO_TEST_CASE(Default){
	Eigen::MatrixXd mat = RX150Kinematics::getTransformMatrixFromDH(0,0,0,0);
	BOOST_CHECK_EQUAL(mat.rows(), 4);
	BOOST_CHECK_EQUAL(mat.cols(), 4);
	boost_check_equal_mat(mat, MatrixXd::Identity(4,4));
}

BOOST_AUTO_TEST_CASE(ThetaOnly1){
	Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0,M_PI/2,0);
	Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
	matPosExpected(0,0) = 0.0;
	matPosExpected(0,1) = -1.0;
	matPosExpected(1,0) = 1.0;
	matPosExpected(1,1) = 0.0;
	boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(ThetaOnly2){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0,-18.3/180.0*M_PI+M_PI/2,0);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(0,0) = -0.31;
    matPosExpected(0,1) = -0.95;
    matPosExpected(1,0) = 0.95;
    matPosExpected(1,1) = -0.31;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(aOnly1){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0.159,0,0);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(0,2) = 0.0;
    matPosExpected(0,3) = 0.159;
    matPosExpected(1,2) = 0.0;
    matPosExpected(1,3) = 0.0;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(aOnly2){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0.15,0,0);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(0,2) = 0.0;
    matPosExpected(0,3) = 0.15;
    matPosExpected(1,2) = 0.0;
    matPosExpected(1,3) = 0.0;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(AlphaOnly1){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(M_PI/2,0,0,0);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(1,1) = 0.0;
    matPosExpected(1,2) = 0.0;
    matPosExpected(2,1) = 1.0;
    matPosExpected(2,2) = 0.0;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(AlphaOnly2){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0,0,0);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(1,1) = 1.0;
    matPosExpected(1,2) = 0.0;
    matPosExpected(2,1) = 0.0;
    matPosExpected(2,2) = 1.0;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(dOnly1){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0,0,0.131);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(2,2) = 1.0;
    matPosExpected(2,3) = 0.131;
    matPosExpected(3,2) = 0.0;
    matPosExpected(3,3) = 1.0;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_CASE(dOnly2){
    Eigen::MatrixXd matPos = RX150Kinematics::getTransformMatrixFromDH(0,0,0,0.1101);
    Eigen::MatrixXd matPosExpected = MatrixXd::Identity(4,4);
    matPosExpected(2,2) = 0.0;
    matPosExpected(2,3) = 0.1101;
    matPosExpected(3,2) = 0.0;
    matPosExpected(3,3) = 1.0;
    boost_check_equal_mat(matPos, matPosExpected);
}

BOOST_AUTO_TEST_SUITE_END()



