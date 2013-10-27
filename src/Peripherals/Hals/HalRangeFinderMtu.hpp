/*
 * HalRangeFinderMtu.hpp
 *
 *  Created on: 2012/11/03
 *      Author: sa
 */

#ifndef HALRANGEFINDERMTU_HPP_
#define HALRANGEFINDERMTU_HPP_

#include "../../iodefine.h"

class HalRangeFinderMtu
{
private:
	static HalRangeFinderMtu *halRangeFinderMtu;
	float pulseWidth;
	float pulseWidthPrev;

	HalRangeFinderMtu();
	~HalRangeFinderMtu();
	void initModuleMtu4();
public:
	const float HZ_SAMPLERATE=10.0;

	static HalRangeFinderMtu* getInstance();

	void start();
	void stop();

	float getPulseWidth();
	float getPulseWidthPrev();

	void updatePulseWidth();
	static void TGIC4();
};


#endif /* HALRANGEFINDERMTU_HPP_ */
