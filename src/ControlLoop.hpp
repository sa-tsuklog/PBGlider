/*
 * ControlLoop.hpp
 *
 *  Created on: 2013/07/27
 *      Author: sa
 */

#ifndef CONTROLLOOP_HPP_
#define CONTROLLOOP_HPP_

#include "ControlLogic.hpp"

class ControlLoop{
public:
	ControlLoop();
	void controlLoop();

	enum Mode{
		modeIdle,
		modeFullManual,
		modeGlide,
	};
	enum Onetime{
		onetimeNone,
		onetimeGyroCalibration,
		onetimeCompassCalibration,
		onetimeAcceloCalibration,
		onetimeTrimCalibration,
		onetimeSetGlideAttitude,
		onetimeSetGlideGain,
		onetimeSetGlideGyroGain,
		onetimeSetFlareParams,
		onetimeHelp,
	};

	enum PrintMode{
		printModeNone,
		printModeAttitude,
		printModeAttitudeEarthFrame,
		printModeAltitude,
		printModeHeadTrack,
	};

	void setMode(ControlLoop::Mode mode){
		this->mode = mode;
	}
	void setOnetime(ControlLoop::Onetime onetime){
		this->onetime = onetime;
	}
	void setPrintmode(ControlLoop::PrintMode printmode){
		this->printmode = printmode;
	}
	void setManualControlParamter(float pitch,float role,float yaw,float throttle,float flaps);



private:
	Mode mode;
	Onetime onetime;
	PrintMode printmode;
	ControlLogic* controlLogic;

	float parseFloat();

	void trimCalibration();
	void servoTrimCalibration();
	void printAttitude();
	void printAttitudeEarthFrame();
	void printAltitude();
	void printHeadTrack();
	void setGlideAttitude();
	void setGlideGain();
	void setGlideGyroGain();
	void setFlareParams();
	void showHelp();
};






#endif /* CONTROLLOOP_HPP_ */
