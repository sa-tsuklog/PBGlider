/*
 * GpsCrReceivedHandler.cpp
 *
 *  Created on: 2013/08/16
 *      Author: sa
 */

#include "GpsCrReceivedHandler.hpp"
#include "../GeneralConfig.hpp"
#include "../Util.hpp"
#include <string.h>
#include <mathf.h>

#define M_EARTH_RADIUS (6371000)

GpsCrReceivedHandler::GpsCrReceivedHandler(HalSci5* receiver){
	this->receiver = receiver;

	valid=0;
	hour=0;
	min=0;
	sec=0.0;
	degLattitudeInt=0;
	degLattitudeFraction=0;		//fraction = degLattitudeFraction * 10^-7
	degLongitudeInt=0;
	degLongitudeFraction=0;		//fraction = degLattitudeFraction * 10^-7
	mpsSpeed=0.0;
	degCourse=0.0;

	degRefLattitudeInt=0;
	degRefLattitudeFraction=0;
	degRefLongitudeInt=0;
	degRefLongitudeFraction=0;

	newDataAvalible=0;
}
GpsCrReceivedHandler::~GpsCrReceivedHandler(){
}

int GpsCrReceivedHandler::isNewDataAvailable(){
	return newDataAvalible;
}
void GpsCrReceivedHandler::clearIsNewDataAvailable(){
	newDataAvalible = 0;
}

int GpsCrReceivedHandler::isValid(){
	return valid;
}
int GpsCrReceivedHandler::getHour(){
	return hour;
}
int GpsCrReceivedHandler::getMin(){
	return min;
}
float GpsCrReceivedHandler::getSec(){
	return sec;
}
int GpsCrReceivedHandler::getDegLattitudeInt(){
	return degLattitudeInt;
}
int GpsCrReceivedHandler::getDegLattitudeFraction(){
	return degLattitudeFraction;
}
int GpsCrReceivedHandler::getDegLongitudeInt(){
	return degLongitudeInt;
}
int GpsCrReceivedHandler::getDegLongitudeFraction(){
	return degLongitudeFraction;
}
float GpsCrReceivedHandler::getMpsSpeed(){
	return mpsSpeed;
}
float GpsCrReceivedHandler::getDegCourse(){
	return degCourse;
}
float GpsCrReceivedHandler::getMpsSpeedX(){
	return getMpsSpeed()*cosf(getDegCourse()*QUAT_PI/180);
}
float GpsCrReceivedHandler::getMpsSpeedY(){
	return getMpsSpeed()*(-sinf(getDegCourse()*QUAT_PI/180));
}
void GpsCrReceivedHandler::setRefPosition(){
	degRefLattitudeInt=degLattitudeInt;
	degRefLattitudeFraction=degLattitudeFraction;
	degRefLongitudeInt=degLongitudeInt;
	degRefLongitudeFraction=degLongitudeFraction;
}

void GpsCrReceivedHandler::resetRefPosition(){
	degRefLattitudeInt = 0;
	degRefLattitudeFraction=0;
	degRefLongitudeInt=0;
	degRefLongitudeFraction=0;
}

float GpsCrReceivedHandler::getM_RelativePosX(){ //north/south
	if(degRefLattitudeInt == 0){
		setRefPosition();
	}

	int degRelativeInt = degLattitudeInt-degRefLattitudeInt;
	int degRelativeFraction = degLattitudeFraction-degRefLattitudeFraction;

	float degRelative = (degRelativeInt*10000000 + degRelativeFraction)*0.0000001;

//	stdout->putString("rel");
//	stdout->putFloat(degRelative,8);
//	stdout->putString("relInt:");
//	stdout->putFloat(degRelativeInt);
//	stdout->putString("relFraction:");
//	stdout->putFloat(degRelativeFraction);
//	stdout->putString("\n\r");


	return degRelative * M_EARTH_RADIUS * QUAT_PI /180.0;
}

float GpsCrReceivedHandler::getM_RelativePosY(){ //west/east
	int degRelativeInt = degLongitudeInt-degRefLongitudeInt;
	int degRelativeFraction = degLongitudeFraction-degRefLongitudeFraction;

	float degRelative = - (degRelativeInt*10000000 + degRelativeFraction)*0.0000001;

	return degRelative * M_EARTH_RADIUS * cosf(degLattitudeInt/180*QUAT_PI) * QUAT_PI /180.0;
}

int GpsCrReceivedHandler::decodeTime(char* message){
	hour = (message[0]-'0') * 10 + (message[1]-'0');
	if(24<hour){
		hour -=24;
	}
	min = (message[2]-'0')*10 + (message[3]-'0');
	sec = float((message[4]-'0')*10 + (message[5]-'0'));

	int index = 7;
	float digit = 0.1;
	while(message[index]!=','){
		sec += (message[index]-'0')*digit;
		digit *= 0.1;
		index++;
	}
	return index+1;
}
int GpsCrReceivedHandler::decodeIsValid(char* message){
	if(message[0] == 'A'){
		valid = 1;
	}else{
		valid = 0;
	}
	return 2;
}
int GpsCrReceivedHandler::decodeLattitude_Longitude(char* message,int* degInt,int* degFraction){
	int buf=0;
	int index=0;
	int tmpDeg;
	int tmpMin_x10_7;
	int digit = 10000000;

	while(message[index] != '.'){
		buf*=10;
		buf += (message[index] - '0');
		index++;
	}
	index++;			//skip '.'
	tmpDeg = buf/100;
	tmpMin_x10_7 = buf%100;
	tmpMin_x10_7 *= digit;

	while(message[index] != ','){
		digit /= 10;
		tmpMin_x10_7 += (message[index] - '0') * digit;
		index++;
	}

	*degInt= tmpDeg;
	*degFraction = tmpMin_x10_7/60;

	return index+1;
}
float GpsCrReceivedHandler::knotToMps(float knot){
	return knot * 0.514444444;
}
int GpsCrReceivedHandler::decodeSpeed(char* message){
	int index = 0;
	float knotSpeed = 0;
	float digit = 0.1;

	while(message[index] != '.'){
		knotSpeed *=10;
		knotSpeed += message[index]-'0';
		index++;
	}
	index++; //'.'

	while(message[index] != ','){
		knotSpeed += (message[index]-'0') * digit;
		digit*=0.1;
		index++;
	}
	index++;	//','

	mpsSpeed = knotToMps(knotSpeed);

	return index;
}
int GpsCrReceivedHandler::decodeCourse(char* message){
	int index = 0;
	float tpmDegCourse = 0;
	float digit = 0.1;
	int sign = 1;

	if(message[0]=='-'){
		sign = -1;
		index++;
	}

	while(message[index] != '.'){
		tpmDegCourse *=10;
		tpmDegCourse += message[index]-'0';
		index++;
	}
	index++; 	//'.'

	while(message[index] != ','){
		tpmDegCourse += (message[index]-'0') * digit;
		digit*=0.1;
		index++;
	}
	index++;	//','

	degCourse = sign*tpmDegCourse;

	return index;
}

void GpsCrReceivedHandler::decodeGPRMC(char* line){
	int index=7;
	index += decodeTime(line+index);
	index += decodeIsValid(line+index);
	if(!valid){
		return;
	}
	index += decodeLattitude_Longitude(line+index,&degLattitudeInt,&degLattitudeFraction);
	index += 2;	//"N,". northern hemisphere only.
	index += decodeLattitude_Longitude(line+index,&degLongitudeInt,&degLongitudeFraction);
	index += 2; //"E,". east longitude only.
	index += decodeSpeed(line+index);
	index += decodeCourse(line+index);

//	stdout->putInt(degLattitudeInt);
//	stdout->putInt(degLattitudeFraction);
//	stdout->putString(",");
//	stdout->putInt(degLongitudeInt);
//	stdout->putInt(degLongitudeFraction);
//	stdout->putString(",course:");
//	stdout->putFloat(degCourse);
//	stdout->putString(",speed:");
//	stdout->putFloat(mpsSpeed);
//	stdout->putString(",");
//	stdout->putString("\n\r");
//
//
//
//
//		stdout->putInt(index);
//		stdout->putString("\n\r");
}
void GpsCrReceivedHandler::decodeNMEA(char* line){
	if(strncmp(line,"$GPRMC,",7)==0){
		decodeGPRMC(line);
		if(valid){
			newDataAvalible = 1;
		}

	}else{
		//do nothing.
	}
}



//@override.
void GpsCrReceivedHandler::actionPerformed(){
	decodeNMEA(receiver->getBuffer());
//	stdout->putString(receiver->getBuffer());
//	stdout->putString("\n\r");
}

void GpsCrReceivedHandler::test(){
	degRefLattitudeInt = 140;
	degRefLattitudeFraction = 8630240;
	degRefLongitudeInt =35;
	degRefLongitudeFraction = 7065780;

	degLattitudeInt = 140;
	degLattitudeFraction = 8629520;
	degLongitudeInt = 35;
	degLongitudeFraction = 7137740;



	stdout->putFloat(getM_RelativePosX());
	stdout->putString(",");
	stdout->putFloat(getM_RelativePosY());
	stdout->putString("\n\r");
	Util::myWait(100);
}
