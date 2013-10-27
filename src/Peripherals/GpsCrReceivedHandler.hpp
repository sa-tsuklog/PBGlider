/*
 * GpsCrReceivedHandler.hpp
 *
 *  Created on: 2013/08/15
 *      Author: sa
 */

#ifndef GPSCRRECEIVEDHANDLER_HPP_
#define GPSCRRECEIVEDHANDLER_HPP_

#include "Interfaces/EventHandler.hpp"
#include "Hals/HalSci5.hpp"

// 1/1000 deg = 111m

class GpsCrReceivedHandler:public EventHandler{
private:
	HalSci5* receiver;

	int newDataAvalible;
	int valid;
	int hour,min;
	float sec;
	int degLattitudeInt,degLattitudeFraction;		//fraction = degLattitudeFraction * 10^-7
	int degLongitudeInt,degLongitudeFraction;		//fraction = degLattitudeFraction * 10^-7
	float mpsSpeed;
	float degCourse;

	int degRefLattitudeInt,degRefLattitudeFraction;
	int degRefLongitudeInt,degRefLongitudeFraction;

	int decodeTime(char* message);
	int decodeIsValid(char* message);
	int decodeLattitude_Longitude(char* message,int* degInt,int* degFraction);
	float knotToMps(float knot);
	int decodeSpeed(char* message);
	int decodeCourse(char* message);
	void decodeGPRMC(char* line);
public:
	GpsCrReceivedHandler(HalSci5* receiver);
	~GpsCrReceivedHandler();

	int isNewDataAvailable();
	void clearIsNewDataAvailable();

	int isValid();
	int getHour();
	int getMin();
	float getSec();
	int getDegLattitudeInt();
	int getDegLattitudeFraction();
	int getDegLongitudeInt();
	int getDegLongitudeFraction();
	float getMpsSpeed();
	float getDegCourse();
	float getMpsSpeedX();
	float getMpsSpeedY();
	void setRefPosition();
	void resetRefPosition();
	float getM_RelativePosX();
	float getM_RelativePosY();

	void decodeNMEA(char* line);
	void test();
	//@override.
	void actionPerformed();
};




#endif /* GPSCRRECEIVEDHANDLER_HPP_ */
