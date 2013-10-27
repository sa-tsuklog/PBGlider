#ifndef __STD_SCI
#define __STD_SCI

#include "../iodefine.h"
#include "Interfaces/WrapperBufferedSci.hpp"

class StdSci{
protected:
	WrapperBufferedSci* printer;

	void putIntZeroFill(int num,int digit);
	void myStrCopy(char*src,char* dst);
	int getStrLength(char* string);
	float myRoundf(float f);
	float myRoundf(float f,int fraction);
	int intPow(int x,int y);
	void putZeros(int digit);

	unsigned char hexToChar(char hexcodeH,char hexcodeL);
	int hexToInt(char hexcode);
public:
	StdSci(WrapperBufferedSci* printer);
	~StdSci();

	void putChar(char value);
	void putString(char* stringPtr);
	void putShortHex(short num);
	void putShort(short num);
	void putInt(int num);
	void putInt(char* string,int num);
	void putInt2(int num);
	void putFloat(float f,int digit);
	void putFloat(float f);
};

#endif
