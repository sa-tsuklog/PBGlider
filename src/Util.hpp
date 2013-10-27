#ifndef __UTIL
#define __UTIL

class Util{
public:
	static void myWait(int time);
	static void myWaitShort(int time);
	static void myException(char* message);
	static float limitValue(float value,float limit);
	static float limitValue(float value,float floor,float ceil);
};

#endif
