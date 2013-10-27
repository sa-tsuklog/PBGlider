#include "Servo.hpp"
#include "../Util.hpp"

//ch1: throttle1
//ch2: throttle2
//ch3: pitch1
//ch4: pitch2
//ch5: yaw1
//ch6: yaw2

Servo::Servo(){;
}
Servo::~Servo(){;
}

void Servo::setPitch(float pos){
	pitch = pos;
}
void Servo::setRole(float pos){
	role=pos;
}
void Servo::setYaw(float pos){
	yaw=pos;
}
void Servo::setFlaps(float pos){
	flap=pos;
}
void Servo::setThrottle(float pos){
	throttle=pos;
}

void Servo::nativeSetServos(int ch,float pos){
	setPos(ch,pos);
}

