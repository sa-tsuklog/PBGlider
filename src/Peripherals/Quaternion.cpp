#include "Quaternion.hpp"
#include "mathf.h"
#include "math.h"
#include "../GeneralConfig.hpp"

Quaternion::Quaternion(){
	this->w=this->x=this->y=this->z=0;
}
Quaternion::Quaternion(float w,float x,float y,float z){
	this->w=w;
	this->x=x;
	this->y=y;
	this->z=z;
}

Quaternion::~Quaternion(){
}
void Quaternion::add(Quaternion q){
	this->w+q.w;
	this->x+q.x;
	this->y+q.y;
	this->z+q.z;
}
void Quaternion::mul(Quaternion q){
	float w,x,y,z;
	w=this->w*q.w - this->x*q.x - this->y*q.y - this->z*q.z;
	x=this->w*q.x + q.w*this->x + q.z*this->y - q.y*this->z;
	y=this->w*q.y + q.w*this->y + this->z*q.x - this->x*q.z;
	z=this->w*q.z + q.w*this->z + this->x*q.y - this->y*q.x;

	this->w=w;
	this->x=x;
	this->y=y;
	this->z=z;
}
void Quaternion::mul(float f){
	this->w*=f;
	this->x*=f;
	this->y*=f;
	this->z*=f;
}
void Quaternion::con(){
	this->x*=-1;
	this->y*=-1;
	this->z*=-1;
}
void Quaternion::con(Quaternion q){
	this->w=q.w;
	this->x=-q.x;
	this->y=-q.y;
	this->z=-q.z;
}
float Quaternion::normalize(){
	float norm=sqrtf(this->w*this->w+this->x*this->x+this->y*this->y+this->z*this->z);
	this->w/=norm;
	this->x/=norm;
	this->y/=norm;
	this->z/=norm;

	return norm;
}
void Quaternion::rotate(Quaternion roter){
	float norm;
	Quaternion tmp1=roter;
	tmp1.normalize();
	Quaternion tmp2=tmp1;
	tmp2.con();
	norm = this->normalize();
	this->mul(tmp2);
	tmp1.mul(*this);
	tmp1.mul(norm);
	*this=tmp1;
}
float Quaternion::getRadHeading(){
	Quaternion tmpQ1=Quaternion(*this);
	Quaternion tmpQ2=Quaternion(0.0, 1.0, 0.0, 0.0);
	Quaternion tmpQ3=Quaternion(*this);
	tmpQ1.con();
	tmpQ2.mul(tmpQ1);
	tmpQ3.mul(tmpQ2);

	return atan2f(tmpQ3.y,tmpQ3.x);
}
float Quaternion::getRadPitch(float radHeading){
	Quaternion tmpQ1=Quaternion(cosf(-radHeading/2),0.0f,0.0f,sinf(-radHeading/2));
	Quaternion tmpQ2=Quaternion(0.0f,1.0f,0.0f,0.0f);
	Quaternion tmpQ3=Quaternion(1.0f,0.0f,0.0f,0.0f);

	tmpQ1.mul(*this);
	tmpQ3.con(tmpQ1);
	tmpQ2.mul(tmpQ3);
	tmpQ1.mul(tmpQ2);

	return atan2f(-tmpQ1.z,tmpQ1.x);
}
float Quaternion::getRadRole(float radHeading,float radPitch){
	float role;
	Quaternion tmpQ2 = Quaternion(cosf(-radHeading/2),0.0f,0.0f,sinf(-radHeading/2));
	Quaternion tmpQ3 = Quaternion(cosf(-radPitch/2),0.0f,sinf(-radPitch/2),0.0f);
	tmpQ2.mul(*this);
	tmpQ3.mul(tmpQ2);
	role=atan2f(tmpQ3.x,tmpQ3.w)*2;
	if(role<-QUAT_PI){
		role+=QUAT_PI*2;
	}else if(QUAT_PI<role){
		role-=QUAT_PI*2;
	}

	return role;
}

void Quaternion::getRadPitchRoleHeading(float* pitch, float* role, float* heading){
	*heading = getRadHeading();
	*pitch   = getRadPitch(*heading);
	*role    = getRadRole(*heading,*pitch);
}

float Quaternion::getRadSlant(){
	float radHeading = this->getRadHeading();

	Quaternion tmpQ1=Quaternion(cosf(-radHeading/2),0.0f,0.0f,sinf(-radHeading/2));

	tmpQ1.mul(*this);

	return 2*acosf(tmpQ1.w);

}


float Quaternion::vectDot(Quaternion vect){
	return this->x*vect.x + this->y*vect.y + this->z * vect.z;
}
void Quaternion::vectCross(Quaternion vect){
	float x,y,z;
	x=this->y*vect.z - this->z*vect.y;
	y=this->z*vect.x - this->x*vect.z;
	z=this->x*vect.y - this->y*vect.x;

	this->w=0.0f;
	this->x=x;
	this->y=y;
	this->z=z;
}


void Quaternion::print(){
	stdout->putString("(");
	stdout->putFloat(w);
	stdout->putString(",");
	stdout->putFloat(x);
	stdout->putString(",");
	stdout->putFloat(y);
	stdout->putString(",");
	stdout->putFloat(z);
	stdout->putString(")\n\r");
}

void Quaternion::printPitchRoleHeading(){
	float pitch,role,heading;
	heading=getRadHeading();
	pitch  =getRadPitch(heading);
	role   =getRadRole(heading,pitch);

	stdout->putString("pitch:");
	stdout->putFloat(pitch*180/QUAT_PI);
	stdout->putString("\trole:");
	stdout->putFloat(role*180/QUAT_PI);
	stdout->putString("\theading:");
	stdout->putFloat(heading*180/QUAT_PI);
	stdout->putString("\n\r");
}
