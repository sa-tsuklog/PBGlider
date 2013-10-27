/*
 * Bmp085.hpp
 *
 *  Created on: 2013/08/18
 *      Author: sa
 */

#ifndef BMP085_HPP_
#define BMP085_HPP_

class Bmp085{
private:
	float paPressure;
	float paRefPressure;
	float mHeight;
	float mpsSpeed;
	float degTemp;
	float degRefTemp;

	unsigned char rawPrsTempHi;
	unsigned char rawPrsTempLo;
	unsigned char rawPressureHi;
	unsigned char rawPressureLo;
	unsigned char rawPressureXlo;

	short ac1;
	short ac2;
	short ac3;
	unsigned short ac4;
	unsigned short ac5;
	unsigned short ac6;
	short b1;
	short b2;
	short mb;
	short mc;
	short md;

	int hzUpdateRate;
public:
	Bmp085(int hzUpdateRate);
	~Bmp085();
	void readPressureCalibrationData();
	void measurePrsTemp();
	void readPrsTemp();
	void measurePressure();
	void readPressure();
	void updatePressure();
	float getPaPressure();
	float getPaReferencePressure();
	float getDegTemp();
	float getDegReferenceTemp();
	float getM_Height();
	float getMpsSpeed();
	void setReference();
	void resetReference();
};

#endif /* BMP085_HPP_ */
