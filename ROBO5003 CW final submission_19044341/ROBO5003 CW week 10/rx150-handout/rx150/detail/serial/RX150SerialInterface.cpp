//
// Created by matthias on 11/03/18.
//



/*#if defined(__linux__) || defined(__APPLE__)

#include <fcntl.h>
#include <termios.h>

#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif*/

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>
#include <sstream>

#include <SerialPort.h>
#include <SerialStream.h>
#include <SerialStreamBuf.h>
#include <string>
#include <stdexcept>
#include <math.h>


using namespace LibSerial;
using namespace std;


#include "RX150SerialInterface.h"


uint8_t HEADER = 255;
constexpr uint8_t LENGTH = 13;

constexpr bool enableLogRawData = false;


void logStatus(SerialStream &serial){
	cout << "Opened: " << serial.IsOpen() << endl;
	cout << "Good: " << serial.good() << endl;
	cout << "eof: " << serial.eof() << endl;
	cout << "fail: " << serial.fail() << endl;
	cout << "bad: " << serial.bad() << endl;
}

void logRawData(uint8_t *buffer, size_t length, bool force=false){
	if(!enableLogRawData && !force) return;
	for(uint i=0; i<LENGTH; i++){
		cout << (uint)buffer[i] << " ";
	}
	cout << endl;
}


void putCheckSum(uint8_t *data, uint pos){
	if(255<pos){
		throw invalid_argument("pos too high for checksum");
	}
	uint sum = 0;
	for(uint i=1; i<pos; i++){/// note: skip header byte 0
		sum += (uint)data[i];
		//cout << (uint)data[i] << " ";
	}
	//cout << "=" << sum << endl;
	data[pos] = sum;
}

bool checkPackageOK(uint8_t *buffer, size_t contentLength){
	bool correct = true;
	if(buffer[0] != HEADER){
		cerr << "Message header corrupt" << endl;
		correct = false;
	}
	if(buffer[1] != contentLength+1){
		cerr << "Message length corrupt" << endl;
		correct = false;
	}
	
	uint sum = 0;
	for(uint i=1; i<contentLength; i++){/// note: skip header byte 0
		sum += (uint)buffer[i];
	}
	if((uint8_t)sum != buffer[contentLength]){
		cerr << "Message corrupt: checksum mismatch" << endl;
		cerr << "\t" << (int)((uint8_t)sum) << endl;
		cerr << "\t" << (int)buffer[contentLength] << endl;
		correct = false;
	}

	if(!correct)
		logRawData(buffer,LENGTH, true);

	return correct;
}

void write(SerialStream &serial, uint8_t *buffer, size_t length){
	serial.write(static_cast<char*>(static_cast<void*>(buffer)),length);
}
void read(SerialStream &serial, uint8_t *buffer, size_t length){
	usleep(1*100);
	serial.read(static_cast<char*>(static_cast<void*>(buffer)),length);
	//cout << n << " characters read" << endl;
	/*uint8_t readBytes = 0;
	while(readBytes < length){
		int n = serial.readsome(static_cast<char*>(static_cast<void*>(buffer+n)),length-n);
		cout << n << " characters read" << endl;
		readBytes += n;
	}*/
}


template <typename T>
void setBuffer(uint8_t *buffer, T data){
	*static_cast<T*>(static_cast<void*>(buffer)) = data;
}


void setModelRX150(SerialStream &serial){
	uint8_t writeData[LENGTH];
	writeData[0] = HEADER;
	writeData[1] = LENGTH;
	writeData[2] = 41; // write model
	//writeData[2] = 42; // read model
	writeData[3] = 1; // fixed value

	setBuffer<uint32_t>(writeData+4, 1);

	setBuffer<uint32_t>(writeData+8, 20150);
 
	putCheckSum(writeData,LENGTH-1);

	logRawData(writeData,LENGTH);

	write(serial,writeData,LENGTH);

	uint8_t data[LENGTH];
	read(serial,data,LENGTH);
	logRawData(data,LENGTH);
}

void runEcho(SerialStream &serial){

	uint8_t ECHO_0 = 1;
	uint8_t ECHO_1 = 0;
	
	uint8_t writeData[LENGTH];

	writeData[0] = HEADER;
	writeData[1] = LENGTH;
	writeData[2] = ECHO_0;
	writeData[3] = ECHO_1;

	writeData[4] = 4;
	writeData[5] = 6;
	writeData[6] = 5;
	writeData[7] = 7;
	
	writeData[8] = 8;
	writeData[9] = 9;
	writeData[10] = 10;
	writeData[11] = 11;
	putCheckSum(writeData,LENGTH-1);

	logRawData(writeData,LENGTH);

	write(serial,writeData,LENGTH);

	uint8_t data[LENGTH];
	read(serial,data,LENGTH);
	logRawData(data,LENGTH);

	bool match = true;
	for(uint i=0;i<LENGTH;i++){
		if(writeData[i] != data[i])
			match = false;
	}
	if(match){
		cout << "Echo successful." << endl;
	}
	else{
		cout << "Error: Echo UNSUCCESSFUL." << endl;
		logRawData(writeData,LENGTH,true);
		logRawData(data,LENGTH,true);
	}
}

int readServoRegister(SerialStream &serial, uint8_t servoID, uint16_t registerID){
	uint8_t READ_REG_0 = 12;
	uint8_t READ_REG_1 = 0;
	
	uint8_t writeData[LENGTH];

	writeData[0] = HEADER;
	writeData[1] = LENGTH;
	writeData[2] = READ_REG_0;
	writeData[3] = READ_REG_1;

	writeData[4] = servoID;
	setBuffer<uint16_t>(writeData+5, registerID);
	//writeData[5] = 6;
	//writeData[6] = 5;
	writeData[7] = 0;
	
	writeData[8] = 0;
	writeData[9] = 0;
	writeData[10] = 0;
	writeData[11] = 0;
	
	putCheckSum(writeData,LENGTH-1);

	logRawData(writeData,LENGTH);

	write(serial,writeData,LENGTH);

	uint8_t data[LENGTH];
	read(serial,data,LENGTH);
	logRawData(data,LENGTH);
	checkPackageOK(data, LENGTH-1);

	return *static_cast<int*>(static_cast<void*>(data+8));
}

void writeServoRegister(SerialStream &serial, uint8_t servoID, uint16_t registerID, int32_t value){
	//uint8_t READ_REG_0 = 12;
	//uint8_t READ_REG_1 = 0;
	//uint8_t WRITE_REG_0 = 11;// write with echo
    uint8_t WRITE_REG_0 = 10; // write without echo (save bandwidth)
	uint8_t WRITE_REG_1 = 0;
	
	uint8_t writeData[LENGTH];

	writeData[0] = HEADER;
	writeData[1] = LENGTH;
	writeData[2] = WRITE_REG_0;
	writeData[3] = WRITE_REG_1;

	writeData[4] = servoID;
	setBuffer<uint16_t>(writeData+5, registerID);
	//writeData[5] = 6;
	//writeData[6] = 5;
	writeData[7] = 0;
	
	setBuffer<int32_t>(writeData+8, value);
	
	putCheckSum(writeData,LENGTH-1);

	logRawData(writeData,LENGTH);

	write(serial,writeData,LENGTH);

	/*uint8_t data[LENGTH];
	read(serial,data,LENGTH);
	logRawData(data,LENGTH);

	checkPackageOK(data, LENGTH-1);*/
}



RX150SerialInterface::RX150SerialInterface(int portNumber) {
    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows

    stringstream portS;
    portS << "/dev/ttyACM" << portNumber;

    /*string setLatencyCommand = "setserial " + portS.str() + " low_latency";
    int r = system(setLatencyCommand.c_str());
    if(r){
        cerr << "Warning: could not set serial port to low latency" << endl;
        cerr << "\t" << setLatencyCommand << endl;
        cerr << "\treturn: " << r << endl;
    }

    portHandler = dynamixel::PortHandler::getPortHandler(portS.str().c_str());

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler->openPort()) {
        cout << "Succeeded to open the port " << portS.str() << endl;
    } else {
        cerr << "Failed to open the port " << portS.str() << endl;
        abort();
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        cout << "Succeeded to change the baudrate!" << endl;
    } else {
        cerr << "Failed to change the baudrate!" << endl;
        abort();
    }*/

    this->serial = new SerialStream(); // Note: do not use parametrized constr. and then Open(), will start off stream "bad"
    this->serial->Open(portS.str());
    //this->serial->SetBaudRate( SerialStreamBuf::BaudRateEnum::BAUD_57600 ); // rate used in successful formware flash
    //this->serial->SetBaudRate( SerialStreamBuf::BaudRateEnum::BAUD_1000000 ); // rate used in successful formware flash
    this->serial->SetBaudRate( BaudRate::BAUD_1000000 ); // rate used in successful formware flash (newer libserial version)

    this->serial->SetVMin(13); // set min bytes to read to message length (reading will stop within package and eof stream otherwise)
    this->serial->SetVTime(2);// force blocking for 0.2s for full VMin read

//    string setLatencyCommand = "setserial " + portS.str() + " low_latency";
//    int r = system(setLatencyCommand.c_str());
//    if(r){
//    	cerr << "Warning: could not set serial port to low latency" << endl;
//    	cerr << "\t" << setLatencyCommand << endl;
//    	cerr << "\treturn: " << r << endl;
//    }

    cout << "Setting robot model" << endl;
    setModelRX150(*serial);

    // set baudrate (register 8) to 1M (value '3') between board and servos
    for (int servoID = 1; servoID <= 5; servoID++) {
        writeServoRegister(*serial, servoID, 8, 3);
    }
    // set servo's return delay time to 20usec (10 ticks of 2usec)
    for (int servoID = 1; servoID <= 5; servoID++) {
        writeServoRegister(*serial, servoID, 9, 10);
    }
    // tune down Position D Gain from 4000 (default) to allow slower outside comm frequency without device vibration
    for (int servoID = 1; servoID <= 5; servoID++) {
        writeServoRegister(*serial, servoID, 80, 100);
    }

    initialDataRead = false;

    running = true;
    updateMicros = 40*1000;
    this->torquesEnabled = false;
    //updateMicros = 1000*1000;
    //updateMicros = 10;
    this->updater = thread(&RX150SerialInterface::runUpdates, this);
}

RX150SerialInterface::~RX150SerialInterface() {
    // Close port
    //portHandler->closePort();
    serial->Close();
}

std::shared_ptr<RX150RobotInterface> RX150SerialInterface::create(int portNumber) {
    return std::shared_ptr<RX150RobotInterface>(new RX150SerialInterface(portNumber));
}

Eigen::VectorXd RX150SerialInterface::getCurrentJointAngles() {

    Eigen::VectorXd joints;
    {
        unique_lock<mutex> lk(dataMutex);
        if(initialDataRead){
            joints = this->currentJointAngles;
        }
        else{
            cout << "Waiting for initial data." << endl;
            initialDataReadNotifier.wait(lk);
            cout << "Initial data received." << endl;
            joints = this->currentJointAngles;
            initialDataRead = true;
        }

    }
    //dataMutex.unlock();

    return joints;
}


void RX150SerialInterface::setTargetJointAngles(Eigen::VectorXd targets) {
    if(targets.rows()!=numberOfJoints()){
        throw invalid_argument("RX150RemoteInterface::setTargetJointAngles(): Wrong number of target joint angles");
    }

    unique_lock<mutex> lk(dataMutex);

    targets = targets.cwiseMax(getJointLowerLimits());
    targets = targets.cwiseMin(getJointUpperLimits());

    this->targetJointAngles = targets;
}

Eigen::VectorXd RX150SerialInterface::getTargetJointAngles() {
    Eigen::VectorXd t;
    {
        unique_lock<mutex> lk(dataMutex);
        t = this->targetJointAngles;
    }
    return t;
}

Eigen::VectorXd RX150SerialInterface::getCurrentJointAnglesFromSerial(){
    Eigen::VectorXd angles(5);
    for(int servoID = 1; servoID <=5; servoID++){
        motorPositions[servoID-1] = readServoRegister(*serial,servoID,132);
        angles[servoID-1] = (motorPositions[servoID-1]-2048)/2048.0 * M_PI;
    }
    return angles;
}

void RX150SerialInterface::writeTargetJointAnglesToSerial(Eigen::VectorXd angles) {
    if (angles.size() == 5) {
        enableTorque();
        for (int servoID = 1; servoID <= 5; servoID++) {
            int32_t value = int32_t(angles[servoID - 1] / M_PI * 2048.0) + 2048;
            writeServoRegister(*serial, servoID, 116, value);
        }
    }
    else if(angles.size() == 0){
        disableTorque();
    }
    else{
        std::cerr << "Wrong joint target dimension: " << angles.size() << " (expecting 5)" << std::endl;
    }

}

void RX150SerialInterface::enableTorque(){
    if(!this->torquesEnabled){
        for (int servoID = 1; servoID <= 5; servoID++) {
            writeServoRegister(*serial, servoID, 64, 1);// torque on
        }
        this->torquesEnabled=true;
    }
}
void RX150SerialInterface::disableTorque(){
    if(this->torquesEnabled){
        for (int servoID = 1; servoID <= 5; servoID++) {
            writeServoRegister(*serial, servoID, 64, 0);// torque off
        }
        this->torquesEnabled=false;
    }

}

void RX150SerialInterface::runUpdates(){
    while(running) {
        auto start_time = std::chrono::high_resolution_clock::now();

        Eigen::VectorXd serverCurrentAngles = getCurrentJointAnglesFromSerial();
        //Eigen::VectorXd serverCurrentAngles;


        Eigen::VectorXd velocityLimit(5);
        velocityLimit << 1, 1, 1, 1, 1; // 1 rad / s
        velocityLimit = velocityLimit*0.5;
        Eigen::VectorXd updateLimit = velocityLimit * (updateMicros / 1000000.0);

        Eigen::VectorXd command;
        Eigen::VectorXd last;

        dataMutex.lock();
        this->currentJointAngles = serverCurrentAngles;
        initialDataReadNotifier.notify_all();
        command = this->targetJointAngles;
        last = this->targetJointAnglesLastSent;
        if (command.rows() > 0){
            if (last.rows() == 0) {
                last = currentJointAngles;
            }

            Eigen::VectorXd controlDiff = command - last;
            controlDiff = controlDiff.cwiseMax(-updateLimit);
            controlDiff = controlDiff.cwiseMin(updateLimit);
            command = last + controlDiff;
            this->targetJointAnglesLastSent = command;
        }
        dataMutex.unlock();

        if(command.rows() > 0){
            writeTargetJointAnglesToSerial(command);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto micros = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();

        if(micros < updateMicros){
            usleep( updateMicros - micros  );
        }
        else{
            cerr << "Warning: Serial port update takes too long: " << micros << "microseconds." << endl;
        }
    }
}
