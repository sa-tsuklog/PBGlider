#ifndef HALSERVOMTUS_HPP_
#define HALSERVOMTUS_HPP_

#include "../../iodefine.h"
#include "../Servo.hpp"
#include "../../GeneralConfig.hpp"
#include "../Storage.hpp"

class HalServoMtus:public Servo{
private:
	static const int chNum = 6;
	static const int servoGain=2000;

	short servoPos[chNum];
	short servoTrim[chNum];

	static HalServoMtus* halServoMtus;
	HalServoMtus();
	~HalServoMtus();
	void initMtu6();
	void initMtu7();
	void initMtu8();
	void initMtu9();
	void initMtu10();
	void initPos();
	void initTrim();

	void updateDuty();

	short getTrimmedPos(short trim,float pos);

	void calcServoPos();
public:
	void start();
	void stop();
	static HalServoMtus* getInstance();

	void setPos(int ch,float pos);
	int getChNum(){return chNum;}

	short getTrim(int ch){return servoTrim[ch];}
	void  setTrim(int ch, short trim){servoTrim[ch]=trim;}
	void loadTrim(){Storage::readTrim(servoTrim,chNum);}
	void saveTrim(){Storage::writeTrim(servoTrim,chNum);}

	static void tgic10();

};


#endif /* HALSERVOMTUS_HPP_ */
