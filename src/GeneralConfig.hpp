#ifndef __GENERAL_CONFIG
#define __GENERAL_CONFIG

#include "iodefine.h"
#include "Peripherals/Inu.hpp"
//#include "Peripherals/SensorFusion.hpp"
#include "Peripherals/StdSci.hpp"
//#include "Peripherals/Seeker.hpp"
#include "Peripherals/Hals/HalServoMtus.hpp"
#include "Peripherals/Mpu9150.hpp"
#include "Peripherals/RangeFinder.hpp"
#include "ControlLoop.hpp"
#include "Peripherals/GpsCrReceivedHandler.hpp"

/*
extern Servo* servo;
extern Seeker* seeker;
extern JoypadReceiver* joypad;
extern SensorFusion* sensorFusion;
extern Mpu9150* mpu9150;
extern Inu* inu;
extern ControlSequence* sequence;
*/

extern StdSci* stdout;
extern Mpu9150* mpu9150;
extern Inu* inu;
extern ControlLoop* controlLoop;
extern RangeFinder* rangeFinder;
extern GpsCrReceivedHandler* gps;

class GeneralConfig{
private:
public:
	static const int i2cIpr = 10;
	static const int mpu9150Ipr = 8;
	static const int uartIpr = 14;
	static const int gpsIpr = 9;
	static const int mtuTgiIpr=4;




};

#endif
