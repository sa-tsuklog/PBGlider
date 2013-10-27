#include "Util.hpp"

void Util::myException(char* message){

}
void Util::myWait(int time){
	volatile int i,j;
	for(i=0;i<time;i++){
		for(j=0;j<9598;j++){
		}
	}
}
void Util::myWaitShort(int time){
	volatile int i,j;
	for(i=0;i<time/5;i++){
		for(j=0;j<11;j++){
		}
	}
	
	for(i=0;i<time;i++){
		for(j=0;j<5;j++){
		} 
	}
}
float Util::limitValue(float value,float limit){
	if(value<-limit){
		value=-limit;
	}else if(limit<value){
		value=limit;
	}
	return value;
}
float Util::limitValue(float value,float floor,float ceil){
	if(value<floor){
		value=floor;
	}else if(ceil<value){
		value=ceil;
	}
	return value;
}
