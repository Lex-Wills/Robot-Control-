//
// Created by matthias on 11/03/18.
//

#ifndef RX150_SERIALINTERFACE_H
#define RX150_SERIALINTERFACE_H

#include "../../rx150/RX150RobotInterface.h"

#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <SerialStream.h>

class RX150SerialInterface : public RX150RobotInterface {
public:
    RX150SerialInterface(int portNumber);
    ~RX150SerialInterface();

    static std::shared_ptr<RX150RobotInterface> create(int portNumber);

    Eigen::VectorXd getCurrentJointAngles();

    void setTargetJointAngles(Eigen::VectorXd);

    Eigen::VectorXd getTargetJointAngles();

private:
    Eigen::VectorXd getCurrentJointAnglesFromSerial();
    void writeTargetJointAnglesToSerial(Eigen::VectorXd);
    void enableTorque();
    void disableTorque();

    void runUpdates();

    std::mutex dataMutex;
    std::condition_variable initialDataReadNotifier;

    bool initialDataRead;
    Eigen::VectorXd currentJointAngles;
    Eigen::VectorXd targetJointAngles;
    Eigen::VectorXd targetJointAnglesLastSent;

    bool running;

    unsigned int updateMicros;
    std::thread updater;


    //dynamixel::PortHandler *portHandler;
    //dynamixel::PacketHandler *packetHandler;
    LibSerial::SerialStream *serial;

    int32_t motorPositions[6];
    bool torquesEnabled;


};


#endif //RX150_SERIALINTERFACE_H
