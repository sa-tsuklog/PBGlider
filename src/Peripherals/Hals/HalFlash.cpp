/*
 * HalFlash.cpp
 *
 *  Created on: 2013/02/10
 *      Author: sa
 */

#include "HalFlash.hpp"
#include "../../iodefine.h"
#include "../../Util.hpp"
#include "../../GeneralConfig.hpp"

#define CODEFLASH_BASE 0xFFFC0000
#define DATAFLASH_BASE 0x00100000
#define BANK_SIZE 0x0000800
#define BANK_NUM 16


#define T_DP8   4
#define T_DP128 10
#define T_DE2K 500
#define T_DBC8  1
#define T_DBC2K 2
#define T_DSPD 1
#define T_DSESD1 1
#define T_DSESD2 3
#define T_DSEED 3
#define T_RESW2 1

short HalFlash::blockBuf[1024];
short HalFlash::currentBlock;

void HalFlash::init(){
	int* src;
	int* dst;
	int i;

	currentBlock = 0;


	FLASH.FENTRYR.WORD = 0xAA00;
	FLASH.FCURAME.WORD = 0xC401;

	src = (int*)0xFEFFE000;
	dst = (int*)0x007F8000;

	for(i=0;i<(0xFF000000-0xFEFFE000)/4;i++){
		dst[i]=src[i];
	}

	resetFCU();
	setPclock();

	FLASH.FCURAME.WORD = 0xC400;
	return;
}

void HalFlash::setPclock(){
	int i;
	char* tmp = (char*) CODEFLASH_BASE;
	short* tmp2= (short*) CODEFLASH_BASE;
	FLASH.PCKAR.BIT.PCKA = 0x30;

	*tmp = 0xE9;
	*tmp = 0x03;

	for(i=0;i<3;i++){
		*tmp2=0x0F0F;
	}
	*tmp = 0xD0;

	if(FLASH.FSTATR0.BIT.FRDY=1){
		errorCheck();
	}else{
		resetFCU();
	}
}

void HalFlash::statusClear(){
	char* tmp = (char*)CODEFLASH_BASE;
	FLASH.FENTRYR.WORD = 0xAA80;
	if(FLASH.FSTATR0.BIT.ILGLERR){
		if(FLASH.FASTAT.BYTE!=0x10){
			FLASH.FASTAT.BYTE=0x10;
		}
	}
	*tmp = 0x50;
}

void HalFlash::errorCheck(){
	int error=0;

	if(FLASH.FSTATR0.BIT.PRGERR){
		error=1;

		stdout->putString("write error:");
		stdout->putShortHex(FLASH.FPESTAT.WORD);
		stdout->putString("\n\r");
	}
	if(FLASH.FSTATR0.BIT.ERSERR){
		error = 1;

		stdout->putString("erase error:");
		stdout->putShortHex(FLASH.FPESTAT.WORD);
		stdout->putString("\n\r");
	}
	if(FLASH.FSTATR0.BIT.ILGLERR){
		error =1 ;

		stdout->putString("illegal command\n\r");
	}

	if(error){
		statusClear();
	}
}

void HalFlash::resetFCU(){
	FLASH.FRESETR.BIT.FRESET=1;
	Util::myWait(T_RESW2);
	FLASH.FRESETR.BIT.FRESET=0;

//	stdout->putString("fcu reset.\n\r");
}

void HalFlash::read8Byte(short* buf,int wordSize,int bank,int wordsOffset){
	int i;
	short* tmp=(short*)(DATAFLASH_BASE+bank*BANK_SIZE+wordsOffset*2);

	if(FLASH.FSTATR0.BIT.FRDY=1){
		errorCheck();
	}else{
		resetFCU();
	}

	FLASH.FENTRYR.WORD = 0xAA00;
	FLASH.FWEPROR.BIT.FLWE = 0x02;

	FLASH.FMODR.BIT.FRDMD = 0;
	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
	FLASH.DFLWE0.WORD = 0x1EFF;
	FLASH.DFLWE1.WORD = 0xE1FF;



	for(i=0;i<wordSize;i++){
		buf[i]=tmp[i];
	}

//	stdout->putString("read address:");
//	stdout->putShortHex((short)((int)tmp>>16&0x0000FFFF));
//	stdout->putShortHex((short)((int)tmp&0x0000FFFF));
//	stdout->putString("\n\r");
//	for(i=0;i<wordSize;i++){
//		stdout->putShort(buf[i]);
//		stdout->putString("\t");
//	}
//	stdout->putString("\n\r");

	return;
}
int HalFlash::blankCheck(int bank,int wordsOffset){
	char* tmp = (char*)(DATAFLASH_BASE+bank*BANK_SIZE+wordsOffset*2);
	int result;

	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
	FLASH.DFLWE0.WORD = 0x1EFF;
	FLASH.DFLWE1.WORD = 0xE1FF;

	FLASH.FENTRYR.WORD = 0xAA80;
	FLASH.FMODR.BIT.FRDMD=1;
	FLASH.DFLBCCNT.BIT.BCSIZE = 0;
	FLASH.DFLBCCNT.BIT.BCADR = 0;

	*tmp = 0x71;
	*tmp = 0xD0;

	Util::myWait(T_DBC2K);

	if(FLASH.FSTATR0.BIT.FRDY=1){
		errorCheck();
	}else{
		resetFCU();
	}

	result = FLASH.DFLBCSTAT.BIT.BCST;

	FLASH.FENTRYR.WORD = 0xAA00;

//	stdout->putShortHex((short)((int)tmp>>16&0x0000FFFF));
//	stdout->putShortHex((short)((int)tmp&0x0000FFFF));
//	if(result == 0){
//		stdout->putString(" blank");
//	}else{
//		stdout->putString(" not blank");
//	}
//
//	stdout->putString("\n\r status");
//	stdout->putShortHex(((unsigned short)FLASH.FASTAT.BYTE)&0x00FF);


	stdout->putString("\n\r");


	return result;
}
void HalFlash::write8Byte(short* buf,int bank,int wordsOffset){
	int i;
	int timeout=0;
	char* tmp;
	short* ptr;

	if(BANK_NUM<bank){
		return;
	}

	FLASH.FENTRYR.WORD = 0xAA80;
	FLASH.FWEPROR.BIT.FLWE = 0x01;
	FLASH.FMODR.BIT.FRDMD = 0;
	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
	FLASH.DFLWE0.WORD = 0x1EFF;
	FLASH.DFLWE1.WORD = 0xE1FF;

	tmp = (char*)(DATAFLASH_BASE+bank*BANK_SIZE+wordsOffset*2);
	ptr = (short*)tmp;
	*tmp = 0xE8;
	*tmp = 0x04;

	for(i=0;i<4;i++){
		ptr[i]=buf[i];
	}

	*tmp = 0xD0;

	Util::myWait(T_DP8);

	if(FLASH.FSTATR0.BIT.FRDY==1){
		errorCheck();
	}else{
		resetFCU();

	}

	FLASH.FENTRYR.WORD = 0xAA00;
	FLASH.FWEPROR.BIT.FLWE = 0x02;

//	stdout->putString("write\n\r");

	return;
}

void HalFlash::erase(int bank){
	char* tmp = (char*)(DATAFLASH_BASE+bank*BANK_SIZE);

	if(BANK_NUM<bank){
		return;
	}


	FLASH.FENTRYR.WORD = 0xAA80;
	FLASH.FWEPROR.BIT.FLWE = 0x01;
	FLASH.FMODR.BIT.FRDMD = 0;
	FLASH.DFLRE0.WORD = 0x2DFF;
	FLASH.DFLRE1.WORD = 0xD2FF;
	FLASH.DFLWE0.WORD = 0x1EFF;
	FLASH.DFLWE1.WORD = 0xE1FF;

	*tmp = 0x20;
	*tmp = 0xD0;

	Util::myWait(T_DE2K);

	if(FLASH.FSTATR0.BIT.FRDY==1){
		errorCheck();
	}else{
		resetFCU();

	}

//	stdout->putString("erase ");
//	stdout->putShortHex((short)((int)tmp>>16&0x0000FFFF));
//	stdout->putShortHex((short)((int)tmp&0x0000FFFF));
//	stdout->putString("\n\r");

	FLASH.FENTRYR.WORD = 0xAA00;
	FLASH.FWEPROR.BIT.FLWE = 0x02;

	return;
}

void HalFlash::readBlockToBuffer(){
	for(int i=0;i<BANK_SIZE/8;i++){
		read8Byte(blockBuf,1024,currentBlock,0);
	}
}

void HalFlash::setbank(int bank){
	currentBlock = bank;
	readBlockToBuffer();
}
void HalFlash::finalize(){
	erase(currentBlock);
	for(int i=0;i<BANK_SIZE/8;i++){
		write8Byte(blockBuf+(i*4),currentBlock,i*4);
	}
}
void HalFlash::write(char* buf,int byteOffset,int byteLength){
	char* cBlockBuf = (char*)blockBuf;
	for(int i=0;i<byteLength;i++){
		cBlockBuf[i+byteOffset] = buf[i];
	}
}
void HalFlash::read(char* buf,int byteOffset,int byteLength){
	char* cBlockBuf = (char*)blockBuf;
	for(int i=0;i<byteLength;i++){
		buf[i] = cBlockBuf[i+byteOffset];
	}
}

void HalFlash::debugPrintCurrentBlock(){
	for(int i=0;i<BANK_SIZE/2;i+=16){
		stdout->putShortHex((short)(i*2));
		stdout->putString(":");
		for(int j=0;j<16;j++){
			stdout->putShortHex(blockBuf[i+j]);
			Util::myWait(5);
		}
		stdout->putString("\n\r");
	}
}
