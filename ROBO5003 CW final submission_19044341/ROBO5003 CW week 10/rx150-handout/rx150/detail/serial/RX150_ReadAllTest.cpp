#include <SerialPort.h>
#include <SerialStream.h>
#include <SerialStreamBuf.h>
#include <string>
#include <stdexcept>
#include <iostream>
#include <math.h>


using namespace LibSerial;
using namespace std;


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
	uint8_t WRITE_REG_0 = 11;
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

	uint8_t data[LENGTH];
	read(serial,data,LENGTH);
	logRawData(data,LENGTH);

	checkPackageOK(data, LENGTH-1);
}


int main(){

	SerialStream serial; // Note: do not use parametrized constr. and then Open(), will start off stream "bad"
	serial.Open("/dev/ttyACM0");
	//serial.SetBaudRate( SerialStreamBuf::BaudRateEnum::BAUD_1000000 );
	serial.SetBaudRate( SerialStreamBuf::BaudRateEnum::BAUD_57600 ); // rate used in successful formware flash

	//serial.SetFlowControl( SerialStreamBuf::FlowControlEnum::FLOW_CONTROL_HARD );

	cout << serial.VMin() << endl;
	serial.SetVMin(13); // set min bytes to read to message length (reading will stop within package and eof stream otherwise)
	cout << serial.VMin() << endl;
	cout << serial.VTime() << endl;
	serial.SetVTime(2);// force blocking for 0.2s for full VMin read 
	cout << serial.VTime() << endl;
	

	string setLatencyCommand = "setserial " + string("/dev/ttyACM0") + " low_latency";
	int r = system(setLatencyCommand.c_str());
	if(r){
		cerr << "Warning: could not set serial port to low latency" << endl;
		cerr << "\t" << setLatencyCommand << endl;
		cerr << "\treturn: " << r << endl;
	}

	cout << "Setting robot model" << endl;
	setModelRX150(serial);


	cout << "Turning LEDs ON" << endl;
	for(int s=1;s<7;s++){
		writeServoRegister(serial,s,65,1); // LED on
		usleep(10*1000);
	}
	usleep(1000*1000);
	cout << "Turning LEDs OFF" << endl;
	for(int s=1;s<7;s++){
		writeServoRegister(serial,s,65,0); // LED on
		usleep(10*1000);
	}
	
	/*cout << "Switching torque ON" << endl;
	for(int servoID = 1; servoID <=7; servoID++){
		writeServoRegister(serial,servoID,64,1); // torque on
		int on = readServoRegister(serial,1,64); // torque on?
		cout << "Servo " << servoID << " on: " << on << endl;
	}*/
	cout << "Switching torque OFF" << endl;
	for(int servoID = 1; servoID <=6; servoID++){
		writeServoRegister(serial,servoID,64,0); // torque on
		int on = readServoRegister(serial,servoID,64); // torque on?
		cout << "Servo " << servoID << " on: " << on << endl;
	}

	cout << "Reading joint current" << endl;
	for(uint t=0; t<100; t++){
		for(int servoID = 1; servoID <=6; servoID++){
			int value = readServoRegister(serial,servoID,132);
			double rad = (value-2048)/2048.0 * M_PI; 
			cout<<rad<<"\t";// << endl;
		}
		cout << endl;

		//logStatus(serial);

		runEcho(serial);
		
		usleep(100*1000);
	}

	/*cout << "Switching torque OFF" << endl;
	for(int servoID = 1; servoID <=7; servoID++){
		writeServoRegister(serial,1,64,0); // torque on
		int on = readServoRegister(serial,1,64); // torque on?
		cout << "Servo " << servoID << " on: " << on << endl;
	}*/	

	logStatus(serial);

	runEcho(serial);

	serial.Close();

}

