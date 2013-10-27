/*
 * BMP085.cpp
 *
 *  Created on: 2013/08/18
 *      Author: sa
 */

#include "Bmp085.hpp"
#include "Hals/HalI2C.hpp"
#include <mathf.h>
#include "../Util.hpp"

#define PRS_ADDR (0x77<<1)

#define PRS_AC1_HI (0xAA)
#define PRS_AC1_LO (0xAB)
#define PRS_AC2_HI (0xAC)
#define PRS_AC2_LO (0xAD)
#define PRS_AC3_HI (0xAE)
#define PRS_AC3_LO (0xAF)
#define PRS_AC4_HI (0xB0)
#define PRS_AC4_LO (0xB1)
#define PRS_AC5_HI (0xB2)
#define PRS_AC5_LO (0xB3)
#define PRS_AC6_HI (0xB4)
#define PRS_AC6_LO (0xB5)

#define PRS_B1_HI (0xB6)
#define PRS_B1_LO (0xB7)
#define PRS_B2_HI (0xB8)
#define PRS_B2_LO (0xB9)

#define PRS_MB_HI (0xBA)
#define PRS_MB_LO (0xBB)
#define PRS_MC_HI (0xBC)
#define PRS_MC_LO (0xBD)
#define PRS_MD_HI (0xBE)
#define PRS_MD_LO (0xBF)

#define PRS_MEASURE_START (0xF4)

#define PRS_TMP_HI (0xF6)
#define PRS_TMP_LO (0xF7)

#define PRS_PRS_HI (0xF6)
#define PRS_PRS_LO (0xF7)
#define PRS_PRS_XLO (0xF8)

#define PRS_OSS (3)

Bmp085::Bmp085(int hzUpdateRate){
	this->hzUpdateRate = hzUpdateRate;

	ac1=0;
	ac2=0;
	ac3=0;
	ac4=0;
	ac5=0;
	ac6=0;
	b1=0;
	b2=0;
	mb=0;
	mc=0;
	md=0;

	mHeight = 0;
	mpsSpeed = 0;

	paRefPressure = 0;
	degRefTemp = 0;
}

void Bmp085::readPressureCalibrationData(){
	unsigned char buf[22];
	for(int i=0;i<22;i++){
		buf[i]=0;
	}

	I2C::i2cDevRead(PRS_ADDR, PRS_AC1_HI, &(buf[0]));
	I2C::i2cDevRead(PRS_ADDR, PRS_AC1_LO, &(buf[1]));

	//Util::myWait(50);

	I2C::i2cDevRead(PRS_ADDR, PRS_AC2_HI,&(buf[2]));
	I2C::i2cDevRead(PRS_ADDR, PRS_AC2_LO,&(buf[3]));

	//Util::myWait(50);


	I2C::i2cDevRead(PRS_ADDR, PRS_AC3_HI,&(buf[4]));
	I2C::i2cDevRead(PRS_ADDR, PRS_AC3_LO,&(buf[5]));

	I2C::i2cDevRead(PRS_ADDR, PRS_AC4_HI,&(buf[6]));
	I2C::i2cDevRead(PRS_ADDR, PRS_AC4_LO,&(buf[7]));

	I2C::i2cDevRead(PRS_ADDR, PRS_AC5_HI,&(buf[8]));
	I2C::i2cDevRead(PRS_ADDR, PRS_AC5_LO,&(buf[9]));

	I2C::i2cDevRead(PRS_ADDR, PRS_AC6_HI,&(buf[10]));
	I2C::i2cDevRead(PRS_ADDR, PRS_AC6_LO,&(buf[11]));

	I2C::i2cDevRead(PRS_ADDR, PRS_B1_HI,&(buf[12]));
	I2C::i2cDevRead(PRS_ADDR, PRS_B1_LO,&(buf[13]));

	I2C::i2cDevRead(PRS_ADDR, PRS_B2_HI,&(buf[14]));
	I2C::i2cDevRead(PRS_ADDR, PRS_B2_LO,&(buf[15]));


	I2C::i2cDevRead(PRS_ADDR, PRS_MB_HI,&(buf[16]));
	I2C::i2cDevRead(PRS_ADDR, PRS_MB_LO,&(buf[17]));

	I2C::i2cDevRead(PRS_ADDR, PRS_MC_HI,&(buf[18]));
	I2C::i2cDevRead(PRS_ADDR, PRS_MC_LO,&(buf[19]));

	I2C::i2cDevRead(PRS_ADDR, PRS_MD_HI,&(buf[20]));
	I2C::i2cDevRead(PRS_ADDR, PRS_MD_LO,&(buf[21]));

	while(I2C::isBufferEmpty()!=1){}

	ac1=(((short)buf[0]<<8)) | ((short)buf[1]);
	ac2=(((short)buf[2]<<8)) | ((short)buf[3]);
	ac3=(((short)buf[4]<<8)) | ((short)buf[5]);
	ac4=(((unsigned short)buf[6]<<8)) | ((unsigned short)buf[7]);
	ac5=(((unsigned short)buf[8]<<8)) | ((unsigned short)buf[9]);
	ac6=(((unsigned short)buf[10]<<8)) | ((unsigned short)buf[11]);
	b1=(((short)buf[12]<<8)) | ((short)buf[13]);
	b2=(((short)buf[14]<<8)) | ((short)buf[15]);
	mb=(((short)buf[16]<<8)) | ((short)buf[17]);
	mc=(((short)buf[18]<<8)) | ((short)buf[19]);
	md=(((short)buf[20]<<8)) | ((short)buf[21]);

	measurePrsTemp();
	Util::myWait(50);
	readPrsTemp();
	measurePressure();
	Util::myWait(50);
	readPressure();

	while(I2C::isBufferEmpty()!=1){}

	updatePressure();
}

void Bmp085::setReference(){
	paRefPressure = paPressure;
	degRefTemp = degTemp;
}
void Bmp085::resetReference(){
	paRefPressure = 0;
}

void Bmp085::measurePrsTemp(){
	I2C::i2cDevWrite(PRS_ADDR, PRS_MEASURE_START, 0x2E);
}
void Bmp085::readPrsTemp(){
	I2C::i2cDevRead(PRS_ADDR, PRS_TMP_HI,&rawPrsTempHi);
	I2C::i2cDevRead(PRS_ADDR, PRS_TMP_LO,&rawPrsTempLo);
}
void Bmp085::measurePressure(){
	I2C::i2cDevWrite(PRS_ADDR, PRS_MEASURE_START, 0x34+(PRS_OSS<<6));
}
void Bmp085::readPressure(){
	I2C::i2cDevRead(PRS_ADDR, PRS_PRS_HI,&rawPressureHi);
	I2C::i2cDevRead(PRS_ADDR, PRS_PRS_LO,&rawPressureLo);
	I2C::i2cDevRead(PRS_ADDR, PRS_PRS_XLO,&rawPressureXlo);
}

void Bmp085::updatePressure(){

	//test signal
	/*
	float ac1=408;
	float ac2=-72;
	float ac3=-14383;
	float ac4=32741;
	float ac5=32757;
	float ac6=23153;
	float b1=6190;
	float b2=4;
	float mb=-32768;
	float mc=-8711;
	float md=2868;

	float rawPrsTemp=27898;
	float rawPressure=23843;
	*/

	float rawPrsTemp =(((int)rawPrsTempHi)<<8)  |  ((int)rawPrsTempLo);
	float rawPressure=(   (  ((int)rawPressureHi)  <<16)|(  ((int)rawPressureLo)  <<8)|  ((int)rawPressureXlo))   >>(8-PRS_OSS);


	float x1;
	float x2;
	float x3;

	float b3;
	float b4;
	float b5;
	float b6;
	float b7;
	float p;

	float temp;

	x1=(rawPrsTemp-ac6)*ac5/powf(2,15);
	x2=mc*powf(2,11)/(x1+md);
	b5=x1+x2;
	temp=(b5+8)/powf(2,4);

	b6=b5-4000;
	x1=(b2*(b6*b6/powf(2,12)))/powf(2,11);
	x2=ac2*b6/powf(2,11);
	x3=x1+x2;
	b3=((ac1*4+x3)*powf(2,PRS_OSS)+2)/4;
	x1=ac3*b6/powf(2,13);
	x2=(b1*(b6*b6/powf(2,12)))/powf(2,16);
	x3=((x1+x2)+2)/4;
	b4=ac4*(x3+32768)/powf(2,15);
	b7=(rawPressure-b3)*(50000>>PRS_OSS);
	if(b7<0x80000000){p=(b7*2)/b4;}
	else{p=(b7/b4)*2;}

	x1=(p/powf(2,8))*(p/powf(2,8));
	x1=(x1*3038)/powf(2,16);
	x2=(-7357*p)/powf(2,16);
	paPressure=p+(x1+x2+3791)/powf(2,4);
	degTemp = temp;


	float mHeightPrev = mHeight;

	if(paRefPressure==0){
		setReference();
	}
//	float p1 = powf((paPressure/paRefPressure),0.1902);
//	mHeight = 153.8 * (degRefTemp + 273.2) * (1-p1);
	mHeight = (paRefPressure-paPressure)/11.4;
	mpsSpeed = (mHeight-mHeightPrev)*hzUpdateRate;
}

float Bmp085::getPaPressure(){
	return paPressure;
}
float Bmp085::getPaReferencePressure(){
	return paRefPressure;
}
float Bmp085::getDegTemp(){
	return degTemp;
}
float Bmp085::getDegReferenceTemp(){
	return degRefTemp;
}
float Bmp085::getM_Height(){
	return mHeight;
}
float Bmp085::getMpsSpeed(){
	return mpsSpeed;
}
