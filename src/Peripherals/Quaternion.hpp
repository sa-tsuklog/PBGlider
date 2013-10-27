#include "../iodefine.h"

#ifndef __QUARTERNION
#define __QUARTERNION

#define QUAT_PI (3.14159265f)

class Quaternion{
public:
	float w,x,y,z;
	Quaternion();
	Quaternion(float w,float x,float y,float z);
	~Quaternion();
	void add(Quaternion q);
	void mul(Quaternion q);
	void mul(float f);
	void con();
	void con(Quaternion q);
	float normalize();
	void rotate(Quaternion roter);
	float getRadHeading();
	float getRadPitch(float radHeading);
	float getRadRole(float radHeading,float radPitch);
	void getRadPitchRoleHeading(float* pitch, float* role, float* heading);
	float getRadSlant();
	void print();
	void printPitchRoleHeading();

	float vectDot(Quaternion vect);
	void vectCross(Quaternion vect);
};

#endif
