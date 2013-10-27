#ifndef __HAL_SCI0
#define __HAL_SCI0

#include <machine.h>
#include "../../iodefine.h"
#include "../Interfaces/WrapperBufferedSci.hpp"
#include "../Interfaces/WrapperSciReceiver.hpp"

class HalSci0:public WrapperBufferedSci,public WrapperSciReceiver{
private:
	int echo;

	static HalSci0* halSci0;
	HalSci0();
	~HalSci0();
	void sendNext();
	inline void lock();
	inline void unlock();

	void received();
public:
	static HalSci0* getInstance();

	void nativeSend(char c);
	char getchar();

	static void txi();
	static void rxi();
	void start();

	void setEcho(int i);
};

#endif
