
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <../rx150/RX150Kinematics.h>
#include <tcp/RX150RemoteInterface.h>


#include <QApplication>
#include <QWidget>
#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSpinBox>
#include <QStackedWidget>

#include "qt/SliderBridge.h"

using namespace std;
using namespace Eigen;


int main(int argc, char* argv[]){
    QApplication app(argc, argv);

    string remoteHost = "localhost";
    if(argc==2){
        remoteHost = argv[1];
    }
    shared_ptr<RX150RobotInterface> robot = RX150RemoteInterface::create(remoteHost);

    //robot->setTargetJointAngles(angles);
    
    SliderBridge bridge(robot);
    
    QWidget window;
    window.setWindowTitle("RX150 Control");
    
    QVBoxLayout *layout = new QVBoxLayout();
        
    QSlider* sliders[5];
    for(int i=0; i<5; i++){
    	sliders[i] = new QSlider(Qt::Horizontal);
    	sliders[i]->setFocusPolicy(Qt::StrongFocus);
    	sliders[i]->setTickPosition(QSlider::TicksBothSides);
    	sliders[i]->setTickInterval(10);
    	sliders[i]->setValue(50);
    	sliders[i]->setSingleStep(1);
    	string label = string("Joint ")+to_string(i+1);
    	layout->addWidget(new QLabel(QString::fromStdString(label)));
    	layout->addWidget(sliders[i]);
    }
    QObject::connect(sliders[0], &QSlider::valueChanged,
                     &bridge, &SliderBridge::setOne);
    QObject::connect(sliders[1], &QSlider::valueChanged,
                     &bridge, &SliderBridge::setTwo);
    QObject::connect(sliders[2], &QSlider::valueChanged,
                     &bridge, &SliderBridge::setThree);
    QObject::connect(sliders[3], &QSlider::valueChanged,
                     &bridge, &SliderBridge::setFour);
    QObject::connect(sliders[4], &QSlider::valueChanged,
                     &bridge, &SliderBridge::setFive);
    
    window.setLayout(layout);
    window.show();
    return app.exec();

}
