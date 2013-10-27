/*
 * CrReceivedHandler.hpp
 *
 *  Created on: 2013/07/08
 *      Author: sa
 */

#ifndef CRRECEIVEDHANDLER_HPP_
#define CRRECEIVEDHANDLER_HPP_

#include "Interfaces/EventHandler.hpp"
#include "Hals/HalSci0.hpp"
#include <string.h>
#include "Hals/HalServoMtus.hpp"
#include "../ControlLoop.hpp"

class CrReceivedHandler:public EventHandler{
private:
	HalSci0* receiver;

	int hexToInt(char hexcode){
		if('0'<=hexcode && hexcode<='9'){
			return hexcode-'0';
		}else if('A'<=hexcode && hexcode <= 'F'){
			return hexcode - 'A' + 10;
		}else if('a'<=hexcode && hexcode <= 'f'){
			return hexcode - 'a' + 10;
		}else{
			return 0;
		}
	}
	unsigned char hexToChar(char hexcodeH,char hexcodeL){
		int upper,lower;
		upper=hexToInt(hexcodeH);
		lower=hexToInt(hexcodeL);
		return (unsigned char)(upper*16 + lower);
	}
	float parseFloat(char* buf,int* outNextIndex){
		int i=0;
		int stage=0;
		float intPart=0.0;
		float fractionPart=0.0;
		float fractionDigit=1.0;
		int sign;

		if(buf[0]=='-'){
			sign=-1;
		}else{
			sign=1;
		}

		while(1){
			if(buf[i] == ' '||buf[i] == ','||buf[i] == '\n'||buf[i] == '\r'){
				break;
			}else if(buf[i] == '.'){
				stage = 1;
			}else if('0' <= buf[i] && buf[i] <='9'){
				if(stage == 0){ //intger part
					intPart *= 10;
					intPart += buf[i]-'0';
				}else{			//fraction part
					fractionDigit*=0.1;
					fractionPart += fractionDigit*(buf[i]-'0');
				}
			}
			i++;
		}

		*outNextIndex = i;
		return sign*(intPart+fractionPart);
	}

public:
	CrReceivedHandler(HalSci0* receiver){
		this->receiver = receiver;
	}
	~CrReceivedHandler(){
	}
	void actionPerformed(){
		char* buf = receiver->getBuffer();
		int numReceived = receiver->getBufferSize();

		//stdout->putString(buf);
		//stdout->putString("\n\r");

		if(strncmp(buf,"setservo",8)==0){
			float pitch,role,yaw,throttle,flaps;

			unsigned char charPos;
			charPos = hexToChar(buf[9],buf[10]);
			pitch = (charPos-127.0)/128.0;

			charPos = hexToChar(buf[12],buf[13]);
			role = (charPos-127.0)/128.0;

			charPos = hexToChar(buf[15],buf[16]);
			yaw = (charPos-127.0)/128.0;

			charPos = hexToChar(buf[18],buf[19]);
			throttle = (charPos-127.0)/128.0;

			charPos = hexToChar(buf[21],buf[22]);
			flaps = (charPos-127.0)/128.0;

			controlLoop->setManualControlParamter(pitch,role,yaw,throttle,flaps);
		}else if(strncmp(buf,"setmode idle",12)==0){
			controlLoop->setMode(ControlLoop::modeIdle);

		}else if(strncmp(buf,"setmode fullmanual",18)==0){
			controlLoop->setMode(ControlLoop::modeFullManual);

		}else if(strncmp(buf,"setmode glide",13)==0){
			controlLoop->setMode(ControlLoop::modeGlide);

		}else if(strncmp(buf,"set glideattitude",17)==0){
			controlLoop->setOnetime(ControlLoop::onetimeSetGlideAttitude);

		}else if(strncmp(buf,"set glidegain",13)==0){
			controlLoop->setOnetime(ControlLoop::onetimeSetGlideGain);

		}else if(strncmp(buf,"set glidegyrogain",17)==0){
			controlLoop->setOnetime(ControlLoop::onetimeSetGlideGyroGain);

		}else if(strncmp(buf,"set flareparams",15)==0){
			controlLoop->setOnetime(ControlLoop::onetimeSetFlareParams);

		}else if(strncmp(buf,"calibrate gyro",14)==0){
			controlLoop->setOnetime(ControlLoop::onetimeGyroCalibration);

		}else if(strncmp(buf,"calibrate compass",14)==0){
			controlLoop->setOnetime(ControlLoop::onetimeCompassCalibration);

		}else if(strncmp(buf,"calibrate accelo",16)==0){
			controlLoop->setOnetime(ControlLoop::onetimeAcceloCalibration);

		}else if(strncmp(buf,"calibrate trim",14)==0){
			controlLoop->setOnetime(ControlLoop::onetimeTrimCalibration);

		}else if(strncmp(buf,"calibrate servotrim",19)==0){
			controlLoop->setOnetime(ControlLoop::onetimeTrimCalibration);

		}else if(strncmp(buf,"reset inu",9)==0){
			mpu9150->resetKalmanFilter();

		}else if(strncmp(buf,"print none",10)==0){
			controlLoop->setPrintmode(ControlLoop::printModeNone);

		}else if(strncmp(buf,"print attitude earthframe",25)==0){
			controlLoop->setPrintmode(ControlLoop::printModeAttitudeEarthFrame);

		}else if(strncmp(buf,"print attitude",14)==0){
			controlLoop->setPrintmode(ControlLoop::printModeAttitude);

		}else if(strncmp(buf,"print altitude",14)==0){
			controlLoop->setPrintmode(ControlLoop::printModeAltitude);

		}else if(strncmp(buf,"print headtrack",14)==0){
			controlLoop->setPrintmode(ControlLoop::printModeHeadTrack);

		}else if(strncmp(buf,"set echo",8)==0){
			if(buf[9]=='0'){
				HalSci0::getInstance()->setEcho(0);
			}else{
				HalSci0::getInstance()->setEcho(1);
			}
		}else if(strncmp(buf,"help",4)==0){
			controlLoop->setOnetime(ControlLoop::onetimeHelp);

		}else if(strncmp(buf,"\n",1)==0){
			//do nothing.

		}else if(numReceived==0){
			//stdout->putString("\n\r");
		}else{
			stdout->putString("no command match :");
			stdout->putString(buf);
			stdout->putString("\n\r");
		}
		//stdout->putString(buf);
		//stdout->putString("\n\r");
	}
};


#endif /* CRRECEIVEDHANDLER_HPP_ */
