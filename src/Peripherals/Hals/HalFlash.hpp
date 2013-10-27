/*
 * HalFlash.hpp
 *
 *  Created on: 2013/02/10
 *      Author: sa
 */

#ifndef HALFLASH_HPP_
#define HALFLASH_HPP_

class HalFlash{
private:
	static short blockBuf[1024];
	static short currentBlock;

	static void resetFCU();
	static void errorCheck();
	static void statusClear();
	static void setPclock();

	static void readBlockToBuffer();
	static void read8Byte(short* buf,int wordSize,int bank,int wordsOffset);
	static int blankCheck(int bank,int wordsOffset);
	static void write8Byte(short* buf,int bank,int wordsOffset);
	static void erase(int bank);

public:
	static void init();

	static void setbank(int bank);
	static void write(char* buf,int byteOffset,int byteLength);
	static void finalize();
	static void read(char* buf,int byteOffset,int byteLength);

	static void debugPrintCurrentBlock();
};

#endif /* HALFLASH_HPP_ */
