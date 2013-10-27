/*
 * RangeFinder.hpp
 *
 *  Created on: 2012/11/03
 *      Author: sa
 */

#ifndef RANGEFINDER_HPP_
#define RANGEFINDER_HPP_

#include "Hals/HalRangeFinderMtu.hpp"

class RangeFinder{
private:
	HalRangeFinderMtu* halRangeFinderMtu;
	float pulseWidthToM(float pulseWidth);
public:
	RangeFinder(HalRangeFinderMtu* halRangeFinderMtu);
	~RangeFinder();

	void start();
	void stop();

	float getM_Range();
	float getMpsSpeed();
};

#endif /* RANGEFINDER_HPP_ */
