
#define BOOST_TEST_COLOR_OUTPUT 1
#define BOOST_TEST_MODULE JacobianTest
#include <boost/test/unit_test.hpp>



// boost has non-virtual constructors ^^
#pragma GCC diagnostic ignored "-Weffc++"

#include <../rx150/RX150Kinematics.h>

#include "testUtilities.h"

#include <iostream>
#include <sstream>

using namespace Eigen;
using namespace std;


BOOST_AUTO_TEST_SUITE(Jacobian)

BOOST_AUTO_TEST_CASE(NoThrow){
	BOOST_CHECK_NO_THROW(RX150Kinematics::computeJacobianMatrix(Eigen::VectorXd(5)));
}

BOOST_AUTO_TEST_CASE(Throw){
	BOOST_CHECK_THROW(RX150Kinematics::computeJacobianMatrix(Eigen::VectorXd()),std::invalid_argument);
}

BOOST_AUTO_TEST_CASE(ZeroJoints){
    Eigen::MatrixXd J = RX150Kinematics::computeJacobianMatrix(Eigen::VectorXd::Zero(5));

    BOOST_CHECK_EQUAL( J.rows(), 6 );
    BOOST_CHECK_EQUAL( J.cols(), 5 );

    // fist v
//        BOOST_CHECK_CLOSE( J(0,0), 0.0, 1e-8 );
    BOOST_CHECK_SMALL( J(0,0), 1e-8 );
//    BOOST_CHECK( abs(J(0,0))<1e-8 );

    BOOST_CHECK(  J(1,0)  >  0.1);
    BOOST_CHECK(  J(1,0)  <  1);

    BOOST_CHECK_SMALL( J(2,0), 1e-8 );

        // second v
        BOOST_CHECK(  J(0,1)  < 0.0);

        BOOST_CHECK_SMALL( J(1,1), 1e-8 );

        BOOST_CHECK(  J(2,1)  >  0.1);
        BOOST_CHECK(  J(2,1)  <  1);

        Eigen::MatrixXd jWexpected(3,5);
        jWexpected << 0, 0, 0, 0, 1,
                 0 ,-1, -1, -1, 0,
                 1, 0, 0, 0, 0;
        Eigen::MatrixXd jW = J.block(3,0,3,5);

        boost_check_equal_mat(jW,jWexpected, 0.0000001);

}

BOOST_AUTO_TEST_CASE(Jacobian_Size){
    Eigen::MatrixXd mat = RX150Kinematics::computeJacobianMatrix(Eigen::VectorXd::Zero(5));
    BOOST_CHECK_EQUAL(mat.rows(), 6);
    BOOST_CHECK_EQUAL(mat.cols(), 5);
}

BOOST_AUTO_TEST_SUITE_END()

