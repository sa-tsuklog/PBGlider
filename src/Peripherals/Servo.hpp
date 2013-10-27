/*
 * Servo.hpp
 *
 *  Created on: 2012/09/09
 *      Author: sa
 */

#ifndef SERVO_HPP_
#define SERVO_HPP_

class Servo{
private:

protected:
	float role,pitch,yaw,throttle,flap;
public:
	Servo();
	virtual ~Servo();

	void setPitch(float pos);
	void setRole(float pos);
	void setYaw(float pos);
	void setFlaps(float pos);
	void setThrottle(float pos);
	void nativeSetServos(int ch,float pos);

	virtual void setPos(int ch,float pos)=0;
	virtual void start()=0;
	virtual void stop()=0;
};

#endif /* SERVO_HPP_ */
