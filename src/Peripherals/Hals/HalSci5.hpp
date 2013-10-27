/*
 * HalSci5.hpp
 *
 *  Created on: 2012/09/17
 *      Author: sa
 */

#ifndef HALSCI5_HPP_
#define HALSCI5_HPP_

#include <machine.h>
#include "../../iodefine.h"
#include "../Interfaces/WrapperBufferedSci.hpp"
#include "../Interfaces/WrapperSciReceiver.hpp"

class HalSci5:public WrapperBufferedSci,public WrapperSciReceiver{
private:
	static HalSci5* halSci5;
	HalSci5();
	void sendNext();
	inline void lock();
	inline void unlock();

	void received();
public:
	static HalSci5* getInstance();

	void nativeSend(char c);

	static void txi();
	static void rxi();
	void start();
};


#endif /* HALSCI5_HPP_ */
