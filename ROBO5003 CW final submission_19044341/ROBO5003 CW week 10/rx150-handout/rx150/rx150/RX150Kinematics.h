//
// Created by matthias on 11/09/18.
//

#ifndef RX150_KINEMATICS_H
#define RX150_KINEMATICS_H

#include <Eigen/Eigen>

class RX150Kinematics {
public:
	// DH Parameters
	const static double alpha[];
	const static double a[];
	const static double thetaOff[];
	const static double d[];

	// Create a 4x4 homogeneous transformation matrix from Denavit-Hartenberg parameters
	static Eigen::MatrixXd getTransformMatrixFromDH(double alpha, double a, double theta, double d);

	
	// Return full 4x4 homogeneous transformation matrix of end-effector frame for given joint angles
	static Eigen::MatrixXd computeForwardKinematics(Eigen::VectorXd jointAngles);

    static Eigen::MatrixXd computeForwardTransforms(Eigen::VectorXd jointAngles, int tn);

	// Return full 6x5 analytic jacobian matrix for given joint angles
	static Eigen::MatrixXd computeJacobianMatrix(Eigen::VectorXd jointAngles);

};


#endif //RX150_KINEMATICS_H

