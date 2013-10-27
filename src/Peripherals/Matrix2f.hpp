/*
 * Matrix.hpp
 *
 *  Created on: 2013/08/18
 *      Author: sa
 */

#ifndef MATRIX2F_HPP_
#define MATRIX2F_HPP_

#include <stdlib.h>
#include "../Util.hpp"
#include "StdSci.hpp"

extern StdSci* stdout;

class Matrix2f{
private:
	float nums[2][2];
	int row,column;

public:
	Matrix2f(){
		row = 2;
		column = 2;

		nums[0][0] = 1;
		nums[0][1] = 0;
		nums[1][0] = 0;
		nums[1][1] = 1;
	}
	Matrix2f(int row,int column){
		this->row = row;
		this->column = column;

		nums[0][0] = 1;
		nums[0][1] = 0;
		nums[1][0] = 0;
		nums[1][1] = 1;
	}
	Matrix2f(float value,int row,int column){
		this->row = row;
		this->column = column;

		nums[0][0] = value;
		nums[0][1] = value;
		nums[1][0] = value;
		nums[1][1] = value;
	}
	Matrix2f(const Matrix2f& obj){
		this->row = obj.row;
		this->column = obj.column;

		nums[0][0] = obj.nums[0][0];
		nums[0][1] = obj.nums[0][1];
		nums[1][0] = obj.nums[1][0];
		nums[1][1] = obj.nums[1][1];
	}
	~Matrix2f(){
	}
	float getNum(int row,int colmun){
		return nums[row][colmun];
	}
	void setNum(float value,int row,int column){
		nums[row][column] = value;
	}
	int getRow(){
		return row;
	}
	int getColumn(){
		return column;
	}
	Matrix2f add(Matrix2f b){
		if(row != b.getColumn() || column != b.getRow()){
			Util::myException("Matrix error\n\r");
		}

		Matrix2f tmp = Matrix2f();

		for(int i=0;i<row;i++){
			for(int j=0;j<column;j++){
				tmp.setNum(nums[i][j] + b.getNum(i,j) , i,j);
			}
		}
		return tmp;
	}
	Matrix2f sub(Matrix2f b){
		if(row != b.getColumn() || column != b.getRow()){
			Util::myException("Matrix error\n\r");
		}

		Matrix2f tmp = Matrix2f();

		for(int i=0;i<row;i++){
			for(int j=0;j<column;j++){
				tmp.setNum(nums[i][j] - b.getNum(i,j) , i,j);
			}
		}
		return tmp;
	}
	Matrix2f mul(Matrix2f b){
		if(column != b.getRow()){
			Util::myException("Matrix error\n\r");
		}

		Matrix2f res = Matrix2f(0,row,b.getColumn());

		for(int i=0;i<row;i++){
			for(int j=0;j<b.getColumn();j++){
				for(int k=0;k<column;k++){
					res.setNum(nums[i][k]*b.getNum(k,j)+res.getNum(i,j) , i,j);
				}
			}
		}

		return res;
	}
	Matrix2f mul(float b){
		Matrix2f res = Matrix2f(0,row,column);

		for(int i=0;i<row;i++){
			for(int j=0;j<column;j++){
					res.setNum(nums[i][j]*b, i,j);
			}
		}

		return res;
	}
	Matrix2f trans(){
		Matrix2f res = Matrix2f(0,column,row);

		res.setNum(nums[0][0],0,0);
		res.setNum(nums[0][1],1,0);
		res.setNum(nums[1][0],0,1);
		res.setNum(nums[1][1],1,1);

		return res;
	}
	Matrix2f inverse(){
		if(row!=2 || column != 2){
			Util::myException("Matrix error at inverse\n\r");
		}

		Matrix2f res = Matrix2f(0,2,2);

		float det = (nums[0][0]*nums[1][1] - nums[0][1]*nums[1][0]);

		res.setNum( nums[1][1]/det,0,0);
		res.setNum(-nums[0][1]/det,0,1);
		res.setNum(-nums[1][0]/det,1,0);
		res.setNum( nums[0][0]/det,1,1);

		return res;
	}




	void print(){
		for(int i=0;i<row;i++){
			for(int j=0;j<column;j++){
				stdout->putFloat(nums[i][j]);
				stdout->putString("\t");
			}
			stdout->putString("\n\r");
		}
		stdout->putString("\n\r");

	}
};


#endif //MATRIX2F_HPP_
