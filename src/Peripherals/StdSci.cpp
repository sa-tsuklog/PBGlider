/*
 * StdSci.cpp
 *
 *  Created on: 2013/06/23
 *      Author: sa
 */

#include "StdSci.hpp"
#include "mathf.h"
#include "math.h"
#include "stdlib.h"


StdSci::StdSci(WrapperBufferedSci* printer){
	this->printer = printer;
}
StdSci::~StdSci(){
	delete printer;
}

void StdSci::putChar(char c){
	printer->putChar(c);
}

void StdSci::putString(char* string){
	char data;
	while( (data = *(string++)) != 0){
		printer->putChar(data);
	}
}

void StdSci::putShortHex(short value){
	int i;
	char hexdigit;
	printer->putChar(' ');
	printer->putChar(' ');
	printer->putChar('0');
	printer->putChar('x');
	for(i=3;0<=i;i--){
		hexdigit=(char)((value>>(i*4))&0x000F);
		if(hexdigit<10){
			printer->putChar((char)(hexdigit+'0'));
		}else{
			printer->putChar((char)(hexdigit+'A'-10));
		}
	}
	printer->putChar(' ');
}

void StdSci::putShort(short num){
	int i;
	int minusFlag=0;
	char numInChar[7];
	numInChar[6]=0;

	if(num<0){
		minusFlag=1;
	}else{
		num*=-1;
	}

	for(i=0;i<6;i++){
		if(num!=0){
			numInChar[5-i]=((char)('0'-num%10));
		}else{
			if(minusFlag){
				numInChar[5-i]='-';
				minusFlag=0;
			}else{
				numInChar[5-i]=' ';
			}
		}
		num/=10;
	}
	if(numInChar[5]==' '){
		numInChar[5]='0';
	}
	putString(numInChar);
}
void StdSci::putInt(int num){
	int i;
	int minusFlag=0;
	char numInChar[12];
	numInChar[11]=0;

	if(num<0){
		minusFlag=1;
	}else{
		num*=-1;
	}
	for(i=0;i<11;i++){
		if(num!=0){
			numInChar[10-i]=((char)('0'-num%10));
		}else{
			if(minusFlag){
				numInChar[10-i]='-';
				minusFlag=0;
			}else{
				numInChar[10-i]=' ';
			}
		}
		num/=10;
	}
	if(numInChar[10]==' '){
		numInChar[10]='0';
	}
	putString(numInChar);
}
void StdSci::putIntZeroFill(int num,int digit){
	int i;
	char* numInChar=(char*)malloc(sizeof(char)*(digit+1));
	numInChar[digit]=0;

	for(i=0;i<digit;i++){
		numInChar[digit-i-1]=((char)('0'+num%10));
		num/=10;
	}

	putString(numInChar);
	free(numInChar);
}

void StdSci::myStrCopy(char*src,char* dst){
	int i=0;
	while(src[i]!=0){
		dst[i]=src[i];
		i++;
	}
	dst[i]=src[i];
}
int StdSci::getStrLength(char* string){
	int i=0;
	while(string[i]!=0){
		i++;
	}
	return i;
}
void StdSci::putInt(char* string,int num){
	int i;
	int minusFlag=0;
	int strLength=getStrLength(string);
	//char numInChar[14+strLength];
	char* numInChar = (char*)malloc(sizeof(char)*(14+strLength));
	myStrCopy(string,numInChar);
	numInChar[11+strLength]='\n';
	numInChar[12+strLength]='\r';
	numInChar[13+strLength]=0;

	if(num<0){
		minusFlag=1;
	}else{
		num*=-1;
	}
	for(i=0;i<11;i++){
		if(num!=0){
			numInChar[10+strLength-i]=((char)('0'-num%10));
		}else{
			if(minusFlag){
				numInChar[10+strLength-i]='-';
				minusFlag=0;
			}else{
				numInChar[10+strLength-i]=' ';
			}
		}
		num/=10;
	}
	if(numInChar[10+strLength]==' '){
		numInChar[10+strLength]='0';
	}
	putString(numInChar);
	free(numInChar);
}
void StdSci::putInt2(int num){

	int i;
	int minusFlag=0;

	if(num==0){
		putString("0");
		return;
	}

	int digit=1;
	if(num<0){
		digit++;
		minusFlag=1;
	}else{
		num*=-1;
	}

	if(-10<num){
		digit+=1;
	}else if(-100<num){
		digit+=2;
	}else if(-1000<num){
		digit+=3;
	}else if(-10000<num){
		digit+=4;
	}else if(-100000<num){
		digit+=5;
	}else if(-1000000<num){
		digit+=6;
	}else if(-10000000<num){
		digit+=7;
	}else if(-100000000<num){
		digit+=8;
	}else if(-1000000000<num){
		digit+=9;
	}else if(-10000000000<num){
		digit+=10;
	}

	char* string=(char*)malloc(sizeof(char)*digit);

	string[digit-1]=0;

	for(i=0;i<digit;i++){
		string[digit-i-2]=((char)('0'-num%10));
		num/=10;
	}

	if(minusFlag){
		string[0]='-';
	}

	putString(string);
	free(string);

}

float StdSci::myRoundf(float f){
	int minusFlag;
	if(f<0){
		minusFlag=-1;
		f*=-1;
	}else{
		minusFlag=1;
	}

	if(0.5<f-floorf(f)){
		return minusFlag*(floorf(f)+1);
	}else{
		return minusFlag*(floorf(f));
	}
}

float StdSci::myRoundf(float f,int fraction){
    f*=powf(10.0,(float)fraction);
    return myRoundf(f)/powf(10.0,(float)fraction);
}

int StdSci::intPow(int x,int y){
	int i;
	int z=1;
	for(i=0;i<y;i++){
		z*=x;
	}
	return z;
}

unsigned char StdSci::hexToChar(char hexcodeH,char hexcodeL){
	int upper,lower;
	upper=hexToInt(hexcodeH);
	lower=hexToInt(hexcodeL);
	return (unsigned char)(upper*16 + lower);
}
int StdSci::hexToInt(char hexcode){
	if('0'<=hexcode && hexcode<='9'){
		return hexcode-'0';
	}else if('A'<=hexcode && hexcode <= 'F'){
		return hexcode - 'A' + 10;
	}else if('a'<=hexcode && hexcode <= 'f'){
		return hexcode - 'a' + 10;
	}else{
		return 0;
	}
}
void StdSci::putZeros(int digit){
	char* string=(char*)malloc(sizeof(digit+1));
	string[digit]=0;
	for(int i=0;i<digit;i++){
		string[i]='0';
	}
	putString(string);
	free(string);
}
void StdSci::putFloat(float f,int digit){
	int intF;
	int shift;

	if(f==0.0){
		putString("0.");
		putZeros(digit);
		return;
	}
	if(f<0){
		putString("-");
		f*=-1;
	}

	shift = intPow(10,digit);
	intF = (int)myRoundf(f*shift);

	putInt2(intF / shift);

	putString(".");

	//f =f*powf(10.0,(float)digit);
	putIntZeroFill(intF,digit);


}
void StdSci::putFloat(float f){
	putFloat(f,3);
}
