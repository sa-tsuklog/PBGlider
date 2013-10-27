#include "HalServoMtus.hpp"
#include "../StdSci.hpp"

HalServoMtus* HalServoMtus::halServoMtus=new HalServoMtus();

void HalServoMtus::initMtu6(){
	MSTP( MTU6 )=0;

	PORTA.DDR.BIT.B0=1;
	PORTA.DDR.BIT.B2=1;

	MTU6.TCR.BIT.CCLR=3;//synchronous clear
	MTU6.TCR.BIT.CKEG=0;
	MTU6.TCR.BIT.TPSC=2;//PCLK/16

	MTU6.TMDR.BIT.MD=2;//PWM mode 1


	MTU6.TIORH.BIT.IOA=2;
	MTU6.TIORH.BIT.IOB=1;
	MTU6.TIORL.BIT.IOC=2;
	MTU6.TIORL.BIT.IOD=1;

	MTUB.TSYR.BIT.SYNC0=1;


	MTU6.TGRA=0;
	MTU6.TGRB=0;
	MTU6.TGRC=7200;
	MTU6.TGRD=7200;
}
void HalServoMtus::initMtu7(){
	MSTP( MTU7 )=0;

	PORTA.DDR.BIT.B4=1;

	MTU7.TCR.BIT.CCLR=3;//synchronous clear
	MTU7.TCR.BIT.CKEG=0;
	MTU7.TCR.BIT.TPSC=2;//PCLK/16

	MTU7.TMDR.BIT.MD=2;//PWM mode 1

	MTU7.TIOR.BIT.IOA=2;
	MTU7.TIOR.BIT.IOB=1;


	MTUB.TSYR.BIT.SYNC1=1;


	MTU7.TGRA=14400;
	MTU7.TGRB=14400;
}
void HalServoMtus::initMtu8(){
	MSTP( MTU8 )=0;

	PORTA.DDR.BIT.B6=1;

	MTU8.TCR.BIT.CCLR=3;//synchronous clear
	MTU8.TCR.BIT.CKEG=0;
	MTU8.TCR.BIT.TPSC=2;//PCLK/16

	MTU8.TMDR.BIT.MD=2;//PWM mode 1

	MTU8.TIOR.BIT.IOA=2;
	MTU8.TIOR.BIT.IOB=1;


	MTUB.TSYR.BIT.SYNC2=1;


	MTU8.TGRA=21600;
	MTU8.TGRB=21600;
}
void HalServoMtus::initMtu9(){
	MSTP( MTU9 )=0;

	PORTB.DDR.BIT.B0=1;
	PORTB.DDR.BIT.B1=1;

	MTU9.TCR.BIT.CCLR=3;//synchronous clear
	MTU9.TCR.BIT.CKEG=0;
	MTU9.TCR.BIT.TPSC=2;//PCLK/16

	MTU9.TMDR.BIT.MD=2;//PWM mode 1

	MTU9.TIORH.BIT.IOA=2;
	MTU9.TIORH.BIT.IOB=1;
	MTU9.TIORL.BIT.IOC=2;
	MTU9.TIORL.BIT.IOD=1;


	MTUB.TSYR.BIT.SYNC3=1;

	MTU9.TGRA=28800;
	MTU9.TGRB=28800;
	MTU9.TGRC=35000;
	MTU9.TGRD=35000;
}
void HalServoMtus::initMtu10(){
	MSTP( MTU10 )=0;

	PORTB.DDR.BIT.B4=1;
	PORTB.DDR.BIT.B5=1;
	PORTB.DDR.BIT.B6=1;

	MTU10.TCR.BIT.CCLR=5;//clear by compare match to
	MTU10.TCR.BIT.CKEG=0;
	MTU10.TCR.BIT.TPSC=2;//PCLK/16

	MTU10.TMDR.BIT.MD=2;//PWM mode 1

	MTUB.TOER.BIT.OE4A=1;

	MTU10.TIORH.BIT.IOA=2;
	MTU10.TIORH.BIT.IOB=1;


	MTUB.TSYR.BIT.SYNC4=1;


	MTU10.TGRA=42200;
	MTU10.TGRB=42200;

	MTU10.TGRC=60000-1;

	MTU10.TIER.BIT.TGIEC=1;
	IEN(MTU10,TGIC10)=1;
	IPR(MTU10,TGIC10)=GeneralConfig::mtuTgiIpr;
}

HalServoMtus::HalServoMtus(){
	initTrim();
	initPos();

	MSTP(MTUB)=0;
	stop();
	initMtu6();
	initMtu7();
	initMtu8();
	initMtu9();
	initMtu10();
}

HalServoMtus::~HalServoMtus(){
	stop();
}

void HalServoMtus::initPos(){
	for(int i=0;i<chNum;i++){
		setPos(i,0.0);
	}
}
void HalServoMtus::initTrim(){
	servoTrim[0]=4500;
	servoTrim[1]=4500;
	servoTrim[2]=4500;
	servoTrim[3]=4500;
	servoTrim[4]=4500;
	servoTrim[5]=4500;
}


void HalServoMtus::start(){
	loadTrim();
	MTUB.TSTR.BYTE|=0xC7;
}
void HalServoMtus::stop(){
	MTUB.TSTR.BYTE&=0x38;
}

HalServoMtus* HalServoMtus::getInstance(){
	return halServoMtus;
}

short HalServoMtus::getTrimmedPos(short trim,float pos){
	short trimmedPos = trim+((short)(servoGain*pos));
	if(trimmedPos<3300){
		trimmedPos = 3300;
	}else if(5700 < trimmedPos){
		trimmedPos = 5700;
	}
	return trimmedPos;
}

void HalServoMtus::setPos(int ch,float pos){
	servoPos[ch] = getTrimmedPos(servoTrim[ch],pos);
}

void HalServoMtus::calcServoPos(){
	setPos(1, pitch);
	setPos(2, -yaw);
	setPos(3, role);
	setPos(4, role);
	setPos(5, pitch);
	setPos(6, pitch);
}

void HalServoMtus::updateDuty(){
	/*
	MTU3.TGRB	=MTU3.TGRA	+ servoPos[0];
	MTU3.TGRD	=MTU3.TGRC	+ servoPos[1];
	MTU4.TGRB	=MTU4.TGRA	+ servoPos[2];
	*/
	calcServoPos();
	MTU6.TGRB  = MTU6.TGRA  + servoPos[0];
	MTU6.TGRD  = MTU6.TGRC  + servoPos[1];
	MTU7.TGRB  = MTU7.TGRA  + servoPos[2];
	MTU8.TGRB  = MTU8.TGRA  + servoPos[3];
	MTU9.TGRB  = MTU9.TGRA  + servoPos[4];
	MTU9.TGRD  = MTU9.TGRC  + servoPos[5];
	MTU10.TGRB = MTU10.TGRA + servoPos[6];

}

#pragma interrupt HalServoMtus::tgic10(vect=VECT( MTU10, TGIC10 ),enable)
void HalServoMtus::tgic10(){
	//GeneralConfig::timingCheckServoUpdateToggle();
	HalServoMtus::getInstance()->updateDuty();
}
