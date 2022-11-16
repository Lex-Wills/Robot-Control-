  //
// Created by matthias on 11/09/18.
//

#include <iostream>
#include "RX150Kinematics.h"

const double RX150Kinematics::alpha[] = {M_PI/2,0.0,0.0,M_PI/2,0.0};
const double RX150Kinematics::a[] = {0.0,0.159,0.15,0.0,0.0};
const double RX150Kinematics::thetaOff[] = {0.0,-18.3/180.0*M_PI+M_PI/2,+18.3/180.0*M_PI-M_PI,M_PI/2,0.0};
const double RX150Kinematics::d[] = {0.1101,0.0,0.0,0.0,0.131};

Eigen::MatrixXd RX150Kinematics::getTransformMatrixFromDH(double alpha, double a, double theta, double d){
    Eigen::MatrixXd m(4,4);
    
    // these are NOT the correct values to fill the matrix with, but it will allow to see some motion in the 3D visualization
    m = Eigen::MatrixXd::Identity(4,4);
    m(0,0) = cos(theta);
    m(0,1) = -sin(theta) * cos(alpha);
    m(0,2) = sin(theta) * sin(alpha);
    m(0,3) = a * cos(theta);
    m(1,0) = sin(theta);
    m(1,1) = cos(theta) * cos(alpha);
    m(1,2) = -cos(theta) * sin(alpha);
    m(1,3) = a * sin(theta);
    m(2,0) = 0;
    m(2,1) = sin(alpha);
    m(2,2) = cos(alpha);
    m(2,3) = d;
    m(3,0) = 0;
    m(3,1) = 0;
    m(3,2) = 0;
    m(3,3) = 1;
    return m;
}

// Return full 4x4 homogeneous transformation matrix of end-effector frame for given joint angles
Eigen::MatrixXd RX150Kinematics::computeForwardKinematics(Eigen::VectorXd jointAngles){
    if(jointAngles.size()!=5){
        throw std::invalid_argument("Invalid number of joint angles");
    }
    Eigen::MatrixXd T(4,4);

    T = getTransformMatrixFromDH(alpha[0],a[0],thetaOff[0] + jointAngles[0],d[0]);
    for (size_t i=1; i < jointAngles.size(); i++) {
        T *= getTransformMatrixFromDH(alpha[i],a[i],thetaOff[i] + jointAngles[i],d[i]);
    }
    return T;
}

  Eigen::MatrixXd RX150Kinematics::computeForwardTransforms(Eigen::VectorXd jointAngles, int tn){
      if(jointAngles.size()!=5){
          throw std::invalid_argument("Invalid number of joint angles");
      }

      if (tn == 0) {
          return Eigen::Matrix4d :: Identity ();
      }

      Eigen::MatrixXd T(4,4);

      T = getTransformMatrixFromDH(alpha[0],a[0],thetaOff[0] + jointAngles[0],d[0]);
      for (size_t i=1; i < tn; i++) {
          T *= getTransformMatrixFromDH(alpha[i],a[i],thetaOff[i] + jointAngles[i],d[i]);
      }
      return T;
  }


// Return full 6x4 analytic jacobian matrix for given joint angles
  Eigen::MatrixXd RX150Kinematics::computeJacobianMatrix(Eigen::VectorXd jointAngles){
      Eigen::MatrixXd J(6,5);

      std::vector<int> row_ind_up {0, 1, 2};
      std::vector<int> row_ind_down {3, 4, 5};
      std::vector<int> z_ind {2};
      std::vector<int> o_ind {3};

      Eigen::MatrixXd T5 = computeForwardKinematics(jointAngles);
      Eigen::VectorXd o_t5_vector = T5(row_ind_up, o_ind);

      Eigen::VectorXd o_tn_vector = Eigen::VectorXd::Zero(3);
      Eigen::VectorXd z_tn_vector = Eigen::VectorXd::Zero(3);

      for (int tn=0; tn < 5; tn++){
          Eigen::MatrixXd transform_matrix = computeForwardTransforms(jointAngles, tn);
          o_tn_vector << transform_matrix(row_ind_up, o_ind);
          z_tn_vector << transform_matrix(row_ind_up, z_ind);

//          std::cout << z_tn_vector.array() * (o_t5_vector - o_tn_vector).array();
//          std::cout << '\n';

          J(row_ind_up, std::vector<int> {tn}) = z_tn_vector.array() * (o_t5_vector - o_tn_vector).array();
          J(row_ind_down, std::vector<int> {tn}) = z_tn_vector;
      }

      return J;
  }




