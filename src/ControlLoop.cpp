#include <mathf.h>
#include <math.h>
#include "ControlLoop.hpp"
#include "ControlLogic.hpp"
#include "GeneralConfig.hpp"
#include "Util.hpp"
#include "Peripherals/Hals/HalSci0.hpp"
#include "Peripherals/Hals/HalServoMtus.hpp"
#include "Peripherals/OrientationFilter.hpp"

ControlLoop::ControlLoop(){
	//mode = modeGlide;
	//mode = modeFullManual;
	mode = modeIdle;
	onetime = onetimeNone;
	printmode = printModeNone;
	//printmode = printModeAttitude;
	//printmode = printModeHeadTrack;

	controlLogic = new ControlLogic();
}

void ControlLoop::controlLoop(){
	while(true){
		if(onetime == onetimeNone){
			//do nothing.
		}else if(onetime == onetimeGyroCalibration){
			mpu9150->gyroCalibration();
		}else if(onetime == onetimeCompassCalibration){
			mpu9150->compassCalibration();
		}else if(onetime == onetimeAcceloCalibration){
			mpu9150->acceloCalibration();
		}else if(onetime == onetimeTrimCalibration){
			trimCalibration();
		}else if(onetime == onetimeSetGlideAttitude){
			setGlideAttitude();
		}else if(onetime == onetimeSetGlideGain){
			setGlideGain();
		}else if(onetime == onetimeSetGlideGyroGain){
			setGlideGyroGain();
		}else if(onetime == onetimeSetFlareParams){
			setFlareParams();
		}else if(onetime == onetimeHelp){
			showHelp();
		}
		onetime=onetimeNone;

		if(mode == modeIdle){
			//do nothing.
			Util::myWait(20);
		}else if(mode == modeFullManual){
			controlLogic->fullManual();
			Util::myWait(20);
		}else if(mode == modeGlide){
			controlLogic->glide();
			Util::myWait(20);
		}

		if(printmode==printModeNone){
			//do nothing.
		}else if(printmode==printModeAttitude){
			printAttitude();
		}else if(printmode==printModeAttitudeEarthFrame){
			printAttitudeEarthFrame();
		}else if(printmode==printModeAltitude){
			printAltitude();
		}else if(printmode==printModeHeadTrack){
			printHeadTrack();
		}

		PORTD.DR.BIT.B7 ^=1;
	}
}



void ControlLoop::trimCalibration(){
	char c;
	short trim;
	stdout->putString("trim start.\n\r");

	for(int ch = 0;  ch<HalServoMtus::getInstance()->getChNum() ;ch++){
		stdout->putString("ch ");
		stdout->putInt(ch);




		while(true){
			//for(int i=0;i<HalServoMtus::getInstance()->getChNum();i++){
			//				HalServoMtus::getInstance()->nativeSetServos(i,0.0);
			//}
			HalServoMtus::getInstance()->setPitch(0.0);
			HalServoMtus::getInstance()->setRole(0.0);
			HalServoMtus::getInstance()->setYaw(0.0);
			HalServoMtus::getInstance()->setFlaps(0.0);
			HalServoMtus::getInstance()->setThrottle(0.0);


			c = HalSci0::getInstance()->getchar();
			if(c == 'u' || c == 'e'){
				trim = HalServoMtus::getInstance()->getTrim(ch);
				HalServoMtus::getInstance()->setTrim(ch,trim + 10);
			}else if(c == 'U' || c == 'E'){
				trim = HalServoMtus::getInstance()->getTrim(ch);
				HalServoMtus::getInstance()->setTrim(ch,trim + 100);
			}else if(c == 'd'){
				trim = HalServoMtus::getInstance()->getTrim(ch);
				HalServoMtus::getInstance()->setTrim(ch,trim - 10);
			}else if(c == 'D'){
				trim = HalServoMtus::getInstance()->getTrim(ch);
				HalServoMtus::getInstance()->setTrim(ch,trim - 100);
			}else if(c == '\n' || '\r'){
				break;
			}
		}
	}
	HalServoMtus::getInstance()->saveTrim();
	stdout->putString("calibrate done.\n\r");

}

float ControlLoop::parseFloat(){
	int stage=0;
	int numEntered=0;
	float intPart=0.0;
	float fractionPart=0.0;
	float fractionDigit=1.0;
	int sign=1;
	char buf;

	while(1){
		buf = HalSci0::getInstance()->getchar();

		if(buf == ' '||buf == ','||buf == '\n'||buf == '\r'){
			break;
		}else if(buf == '-'){
			sign=-1;
		}else if(buf == '.'){
			stage = 1;
		}else if('0' <= buf && buf <='9'){
			numEntered = 1;
			if(stage == 0){ //intger part
				intPart *= 10;
				intPart += buf-'0';
			}else{			//fraction part
				fractionDigit*=0.1;
				fractionPart += fractionDigit*(buf-'0');
			}
		}
	}

	return sign*(intPart+fractionPart);

}

void ControlLoop::setGlideAttitude(){
	float f;
	stdout->putString("set glide attitude\n\r");

	stdout->putString("pitch[deg] (=");
	stdout->putFloat(controlLogic->degGlidePitch);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->degGlidePitch = f;


	stdout->putString("role[deg] (=");
	stdout->putFloat(controlLogic->degGlideRole);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->degGlideRole = f;

	Storage::writeGlideAttitude(controlLogic->degGlidePitch,controlLogic->degGlideRole);

	stdout->putString("set attitude done.\n\r");
}

void ControlLoop::setGlideGain(){
	float f;
	stdout->putString("set glide gain\n\r");

	stdout->putString("pitch gain (=");
	stdout->putFloat(controlLogic->glidePitchGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glidePitchGain = f;

	stdout->putString("pitch gain floor (=");
	stdout->putFloat(controlLogic->glidePitchGainFloor);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glidePitchGainFloor = f;

	stdout->putString("estimated min speed(=");
	stdout->putFloat(controlLogic->glideMinSpeed);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideMinSpeed = f;



	stdout->putString("role gain (=");
	stdout->putFloat(controlLogic->glideRoleGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideRoleGain = f;


	stdout->putString("heading gain (=");
	stdout->putFloat(controlLogic->glideYawGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideYawGain = f;

	stdout->putString("glide deadtime (=");
	stdout->putFloat(controlLogic->glideDeadtime);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideDeadtime = f;

	Storage::writeGlideGain(controlLogic->glidePitchGain,controlLogic->glidePitchGainFloor,controlLogic->glideMinSpeed,
			controlLogic->glideRoleGain,controlLogic->glideYawGain,controlLogic->glideDeadtime);

	stdout->putString("set glidegain done.\n\r");
}

void ControlLoop::setGlideGyroGain(){
	float f;
	stdout->putString("set glide gyro gain\n\r");

	stdout->putString("pitch gyro gain (=");
	stdout->putFloat(controlLogic->glideGyroPitchGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideGyroPitchGain = f;


	stdout->putString("role gain (=");
	stdout->putFloat(controlLogic->glideGyroRoleGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideGyroRoleGain = f;

	stdout->putString("heading gain (=");
	stdout->putFloat(controlLogic->glideGyroYawGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->glideGyroYawGain = f;


	Storage::writeGlideGyroGain(controlLogic->glideGyroPitchGain,controlLogic->glideGyroRoleGain,controlLogic->glideGyroYawGain);

	stdout->putString("set glidegyrogain done.\n\r");
}

void ControlLoop::setFlareParams(){
	float f;
	stdout->putString("set flare params\n\r");

	stdout->putString("deg flare angle (=");
	stdout->putFloat(controlLogic->degFlareAngle);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->degFlareAngle= f;


	stdout->putString("flare gain (=");
	stdout->putFloat(controlLogic->flareGain);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->flareGain = f;

	stdout->putString("flare start height[m] (=");
	stdout->putFloat(controlLogic->mFlareStartHeight);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->mFlareStartHeight = f;


	stdout->putString("flare complete height[m] (=");
	stdout->putFloat(controlLogic->mFlareCompleteHeight);
	stdout->putString(")\n\r");
	f = parseFloat();

	controlLogic->mFlareCompleteHeight = f;

	Storage::writeFlareParam(controlLogic->degFlareAngle,controlLogic->flareGain,controlLogic->mFlareStartHeight,controlLogic->mFlareCompleteHeight);

	stdout->putString("set flareparams done.\n\r");
}

void ControlLoop::printAltitude(){
	static int printLoopCounter=0;
	printLoopCounter = (printLoopCounter+1)%4;

	if(printLoopCounter==0){
		stdout->putFloat(rangeFinder->getM_Range());
		stdout->putString("[m]\n\r");
	}
}

void ControlLoop::printAttitude(){
	static int printLoopCounter=0;

	printLoopCounter = (printLoopCounter+1)%4;

	if(printLoopCounter==0){

		float gyro[3];
		float acl[3];
		float cmps[3];
		int i;

		for(i=0;i<3;i++){
			gyro[i]=mpu9150->getRpsGyro(i);
			acl[i]=mpu9150->getG_Accel(i);
			cmps[i]=mpu9150->get_uT_Cmps(i);
		}

		float heading=OrientationFilter::getAttitude().getRadHeading();
		float pitch=OrientationFilter::getAttitude().getRadPitch(heading);
		float role=OrientationFilter::getAttitude().getRadRole(heading,pitch);

		//OrientationFilter::getAttitude().print();
		stdout->putString("heading:");
		stdout->putFloat((heading*180.0/3.14));
		stdout->putString(",");
		stdout->putString("pitch:");
		stdout->putFloat((pitch*180.0/3.14));
		stdout->putString(",");
		stdout->putString("role:");
		stdout->putFloat((role*180.0/3.14));
		stdout->putString("acl:");
		/*
		stdout->putString(",");
		stdout->putFloat(acl[0]);
		stdout->putString(",");
		stdout->putFloat(acl[1]);
		stdout->putString(",");
		stdout->putFloat(acl[2]);
		stdout->putString(",");
		*/
		stdout->putString(",");
		stdout->putFloat(acl[0]);
		stdout->putString(",");
		stdout->putFloat(acl[1]);
		stdout->putString(",");
		stdout->putFloat(acl[2]);
		stdout->putString(",");
		//stdout->putString("\n\r");

		stdout->putString("gyro:");
		stdout->putFloat(gyro[0]);
		stdout->putString(",");
		stdout->putFloat(gyro[1]);
		stdout->putString(",");
		stdout->putFloat(gyro[2]);
		stdout->putString(",");
		stdout->putString("cmps:");
		stdout->putFloat(cmps[0]);
		stdout->putString(",");
		stdout->putFloat(cmps[1]);
		stdout->putString(",");
		stdout->putFloat(cmps[2]);
		stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().w);
		//			stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().x);
		//			stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().y);
		//			stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().z);
		//			stdout->putString(",");
		stdout->putString("spd[m/s]:");
		stdout->putFloat(mpu9150->getMpsKalmanSpeed(0));
		stdout->putString(",");
		stdout->putFloat(mpu9150->getMpsKalmanSpeed(1));
		stdout->putString(",");
		stdout->putFloat(mpu9150->getMpsKalmanSpeed(2));
		stdout->putString(",");
		stdout->putString("pos[m]:");
		stdout->putFloat(mpu9150->getM_KalmanPos(0));
		stdout->putString(",");
		stdout->putFloat(mpu9150->getM_KalmanPos(1));
		stdout->putString(",");
		stdout->putFloat(mpu9150->getM_KalmanPos(2));
		stdout->putString(",");
		stdout->putString("gps pos[m]:");
		stdout->putFloat(gps->getM_RelativePosX());
		stdout->putString(",");
		stdout->putFloat(gps->getM_RelativePosY());
		stdout->putString(",\n\r");
	}
}


void ControlLoop::printAttitudeEarthFrame(){
	static int printLoopCounter=0;

	printLoopCounter = (printLoopCounter+1)%4;

	if(printLoopCounter==0){
		Quaternion eAccel = inu->getE_FrameAccel();

		float gyro[3];
		float acl[3];
		float cmps[3];
		int i;

		for(i=0;i<3;i++){
			gyro[i]=mpu9150->getRpsGyro(i);
			acl[i]=mpu9150->getG_Accel(i);
			cmps[i]=mpu9150->get_uT_Cmps(i);
		}

		float heading=OrientationFilter::getAttitude().getRadHeading();
		float pitch=OrientationFilter::getAttitude().getRadPitch(heading);
		float role=OrientationFilter::getAttitude().getRadRole(heading,pitch);

		//OrientationFilter::getAttitude().print();
		stdout->putString("heading:");
		stdout->putFloat((heading*180.0/3.14));
		stdout->putString(",");
		stdout->putString("pitch:");
		stdout->putFloat((pitch*180.0/3.14));
		stdout->putString(",");
		stdout->putString("role:");
		stdout->putFloat((role*180.0/3.14));
		stdout->putString("acl:");
		/*
		stdout->putString(",");
		stdout->putFloat(acl[0]);
		stdout->putString(",");
		stdout->putFloat(acl[1]);
		stdout->putString(",");
		stdout->putFloat(acl[2]);
		stdout->putString(",");
		*/
		stdout->putString(",");
		stdout->putFloat(eAccel.x);
		stdout->putString(",");
		stdout->putFloat(eAccel.y);
		stdout->putString(",");
		stdout->putFloat(eAccel.z-1.0);
		stdout->putString(",");
		//stdout->putString("\n\r");

		stdout->putString("gyro:");
		stdout->putFloat(gyro[0]);
		stdout->putString(",");
		stdout->putFloat(gyro[1]);
		stdout->putString(",");
		stdout->putFloat(gyro[2]);
		stdout->putString(",");
		stdout->putString("cmps:");
		stdout->putFloat(cmps[0]);
		stdout->putString(",");
		stdout->putFloat(cmps[1]);
		stdout->putString(",");
		stdout->putFloat(cmps[2]);
		stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().w);
		//			stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().x);
		//			stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().y);
		//			stdout->putString(",");
		//			stdout->putFloat(OrientationFilter::getAttitude().z);
		//			stdout->putString(",");
		stdout->putString("spd[m/s]:");
		stdout->putFloat(inu->getE_FrameSpeed(0));
		stdout->putString(",");
		stdout->putFloat(inu->getE_FrameSpeed(1));
		stdout->putString(",");
		stdout->putFloat(inu->getE_FrameSpeed(2));
		stdout->putString(",");
		stdout->putString("pos[m]:");
		stdout->putFloat(inu->getE_FramePos(0));
		stdout->putString(",");
		stdout->putFloat(inu->getE_FramePos(1));
		stdout->putString(",");
		stdout->putFloat(inu->getE_FramePos(2));
		stdout->putString(",\n\r");
	}
}

void ControlLoop::printHeadTrack(){
	Quaternion eAccel = inu->getE_FrameAccel();

	stdout->putFloat(OrientationFilter::getAttitude().w,4);
	stdout->putString(",");
	stdout->putFloat(OrientationFilter::getAttitude().x,4);
	stdout->putString(",");
	stdout->putFloat(OrientationFilter::getAttitude().y,4);
	stdout->putString(",");
	stdout->putFloat(OrientationFilter::getAttitude().z,4);
//	stdout->putString(",");
//
//	stdout->putFloat(0,1);
//	stdout->putString(",");
//	stdout->putFloat(0,1);
//	stdout->putString(",");
//	stdout->putFloat(0,1);
//
	stdout->putString("\n\r");
}

void ControlLoop::setManualControlParamter(float pitch,float role,float yaw,float throttle,float flaps){
	controlLogic->pitch=pitch;
	controlLogic->role=role;
	controlLogic->yaw=yaw;
	controlLogic->throttle=throttle;
	controlLogic->flaps=flaps;
}

void ControlLoop::showHelp(){
	stdout->putString("setservo\n\r");
	Util::myWait(5);
	stdout->putString("setmode idle\n\r");
	Util::myWait(5);
	stdout->putString("setmode fullmanual\n\r");
	Util::myWait(5);
	stdout->putString("setmode glide\n\r");
	Util::myWait(5);
	stdout->putString("set glideattitude\n\r");
	Util::myWait(5);
	stdout->putString("set glidegain\n\r");
	Util::myWait(5);
	stdout->putString("set glidegyrogain\n\r");
	Util::myWait(5);
	stdout->putString("set flareparams\n\r");
	Util::myWait(5);
	stdout->putString("calibrate gyro\n\r");
	Util::myWait(5);
	stdout->putString("calibrate compass\n\r");
	Util::myWait(5);
	stdout->putString("calibrate trim\n\r");
	Util::myWait(5);
	stdout->putString("calibrate servotrim\n\r");
	Util::myWait(5);
	stdout->putString("print none\n\r");
	Util::myWait(5);
	stdout->putString("print attitude\n\r");
	Util::myWait(5);
	stdout->putString("set echo\n\r");
	Util::myWait(5);
	stdout->putString("help\n\r");
}
