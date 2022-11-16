
#include <boost/test/unit_test.hpp>
#include <Eigen/Eigen>

#include <iostream>
#include <sstream>

using namespace Eigen;


void boost_check_equal_mat(MatrixXd result, MatrixXd expected, double tolerance=0.0001){
	BOOST_CHECK_EQUAL(result.rows(), expected.rows());
	BOOST_CHECK_EQUAL(result.cols(), expected.rows());
	MatrixXd difference = result - expected;
	double dist = difference.norm();
	if(tolerance < dist){
		std::stringstream msg;
		msg << "Expected " << expected << ", but got " << result << ". (difference " << dist << " > tolerance " << tolerance << ")";
		BOOST_FAIL(msg.str());
	}
}
