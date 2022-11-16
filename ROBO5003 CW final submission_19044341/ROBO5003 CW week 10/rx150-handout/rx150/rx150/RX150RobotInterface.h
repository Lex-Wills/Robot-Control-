//
// Created by matthias on 11/03/18.
//

#ifndef RX150_ROBOTINTERFACE_H
#define RX150_ROBOTINTERFACE_H

#include <Eigen/Eigen>

class RX150RobotInterface {
public:
    virtual Eigen::VectorXd getCurrentJointAngles() = 0;

    virtual void setTargetJointAngles(Eigen::VectorXd) = 0;

    virtual Eigen::VectorXd getTargetJointAngles() = 0;

    unsigned int numberOfJoints(){
        return 5;
    }

    Eigen::VectorXd getJointLowerLimits(){
        Eigen::VectorXd lim(5);
        lim << -180.0/180.0*M_PI , -106.0/180.0*M_PI , -102.0/180.0*M_PI , -100.0/180.0*M_PI, -180.0/180.0*M_PI;
        return lim;
    }

    Eigen::VectorXd getJointUpperLimits(){
        Eigen::VectorXd lim(5);
        lim <<  180.0/180.0*M_PI , 100.0/180.0*M_PI , 95.0/180.0*M_PI , 123.0/180.0*M_PI, 180.0/180.0*M_PI;
        return lim;
    }

};


#endif //RX150_ROBOTINTERFACE_H
