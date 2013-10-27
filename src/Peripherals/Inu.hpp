/*
 * Inu.hpp
 *
 *  Created on: 2013/02/08
 *      Author: sa
 */

#ifndef INU_HPP_
#define INU_HPP_

#include "Quaternion.hpp"

class Inu{
private:
	float eFramePos[3];
	float eFrameSpeed[3];

	float posAutoZeroCoef;	//0 to 1
	float spdAutoZeroCoef;	//0 to 1
public:
	Inu();
	~Inu();
	void update();
	void reset();
	float getE_FramePos(int axis);
	float getE_FrameSpeed(int axis);
	Quaternion getB_FrameSpeed();
	Quaternion getE_FrameAccel();
};

#endif /* INU_HPP_ */
