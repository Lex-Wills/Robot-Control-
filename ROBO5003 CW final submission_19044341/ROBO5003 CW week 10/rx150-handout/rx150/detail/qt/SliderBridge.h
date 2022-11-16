

#ifndef _SliderBridge_H_
#define _SliderBridge_H_

#include <QObject>
#include <Eigen/Eigen>
#include <tcp/RX150RemoteInterface.h>

class SliderBridge : public QObject
{
    Q_OBJECT

public:
    SliderBridge(std::shared_ptr<RX150RobotInterface> robot);

public slots:
    void setOne(double value);
    void setTwo(double value);
    void setThree(double value);
    void setFour(double value);
    void setFive(double value);
    

private:
    void send();

    std::shared_ptr<RX150RobotInterface> robot;
    Eigen::VectorXd q;
};

#endif
