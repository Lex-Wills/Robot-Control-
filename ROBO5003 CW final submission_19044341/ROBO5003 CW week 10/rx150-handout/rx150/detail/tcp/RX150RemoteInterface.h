//
// Created by matthias on 11/03/18.
//

#ifndef RX150_SERVERCLIENT_AX18REMOTEINTERFACE_H
#define RX150_SERVERCLIENT_AX18REMOTEINTERFACE_H

#include "../../rx150/RX150RobotInterface.h"

#include <memory>
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>

class RX150RemoteInterface : public RX150RobotInterface {
public:
    static std::shared_ptr<RX150RobotInterface> create(std::string host = "localhost");

    RX150RemoteInterface(std::string host);
    ~RX150RemoteInterface();

    Eigen::VectorXd getCurrentJointAngles();

    void setTargetJointAngles(Eigen::VectorXd);

    Eigen::VectorXd getTargetJointAngles();

private:
    Eigen::VectorXd getCurrentJointAnglesFromServer();
    void writeTargetJointAnglesToServer(Eigen::VectorXd);

    void runUpdates();

    boost::asio::ip::tcp::socket *socket;

    std::mutex dataMutex;
    std::condition_variable initialDataReadNotifier;
    bool initialDataRead;
    Eigen::VectorXd currentJointAngles;
    Eigen::VectorXd targetJointAngles;

    bool running;

    unsigned int updateMicros;
    std::thread updater;
};


#endif //RX150_REMOTEINTERFACE_H
