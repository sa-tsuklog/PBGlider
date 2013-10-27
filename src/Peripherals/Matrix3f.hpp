/*
 * Matrix3f.hpp
 *
 *  Created on: 2013/07/29
 *      Author: sa
 */

#ifndef MATRIX3F_HPP_
#define MATRIX3F_HPP_

#include <stdlib.h>
#include "../Util.hpp"
#include "StdSci.hpp"


extern StdSci* stdout;

class Matrix3f {
private:
    float nums[3][3];
    int row, column;

public:

    Matrix3f() {
        row = 3;
        column = 3;

        nums[0][0] = 1;
        nums[0][1] = 0;
        nums[0][2] = 0;
        nums[1][0] = 0;
        nums[1][1] = 1;
        nums[1][2] = 0;
        nums[2][0] = 0;
        nums[2][1] = 0;
        nums[2][2] = 1;
    }

    Matrix3f(int row, int column) {
        this->row = row;
        this->column = column;

        nums[0][0] = 1;
        nums[0][1] = 0;
        nums[0][2] = 0;
        nums[1][0] = 0;
        nums[1][1] = 1;
        nums[1][2] = 0;
        nums[2][0] = 0;
        nums[2][1] = 0;
        nums[2][2] = 1;
    }

    Matrix3f(float value, int row, int column) {
        this->row = row;
        this->column = column;

        nums[0][0] = value;
        nums[0][1] = value;
        nums[0][2] = value;
        nums[1][0] = value;
        nums[1][1] = value;
        nums[1][2] = value;
        nums[2][0] = value;
        nums[2][1] = value;
        nums[2][2] = value;
    }

    Matrix3f(const Matrix3f& obj) {
        this->row = obj.row;
        this->column = obj.column;

        nums[0][0] = obj.nums[0][0];
        nums[0][1] = obj.nums[0][1];
        nums[0][2] = obj.nums[0][2];
        nums[1][0] = obj.nums[1][0];
        nums[1][1] = obj.nums[1][1];
        nums[1][2] = obj.nums[1][2];
        nums[2][0] = obj.nums[2][0];
        nums[2][1] = obj.nums[2][1];
        nums[2][2] = obj.nums[2][2];
    }

    ~Matrix3f() {
    }

    float getNum(int row, int colmun) {
        return nums[row][colmun];
    }

    void setNum(float value, int row, int column) {
        nums[row][column] = value;
    }

    int getRow() {
        return row;
    }

    int getColumn() {
        return column;
    }

    Matrix3f add(Matrix3f b) {
        if (row != b.getRow() || column != b.getColumn()) {
            Util::myException("matrix error\n\r");
        }

        Matrix3f tmp = Matrix3f(row,column);

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < column; j++) {
                tmp.setNum(nums[i][j] + b.getNum(i, j), i, j);
            }
        }
        return tmp;
    }

    Matrix3f sub(Matrix3f b) {
        if (row != b.getRow() || column != b.getColumn()) {
            Util::myException("matrix error\n\r");
        }

        Matrix3f tmp = Matrix3f(row,column);

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < column; j++) {
                tmp.setNum(nums[i][j] - b.getNum(i, j), i, j);
            }
        }
        return tmp;
    }

    Matrix3f mul(Matrix3f b) {
        if (column != b.getRow()) {
            Util::myException("matrix error\n\r");
        }

        Matrix3f res = Matrix3f(0, row, b.getColumn());

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < b.getColumn(); j++) {
                for (int k = 0; k < column; k++) {
                    res.setNum(nums[i][k] * b.getNum(k, j) + res.getNum(i, j), i, j);
                }
            }
        }

        return res;
    }

    Matrix3f mul(float b) {
        Matrix3f res = Matrix3f(0, row, column);

        for (int i = 0; i < row; i++) {
            for (int j = 0; j < column; j++) {
                res.setNum(nums[i][j] * b, i, j);
            }
        }

        return res;
    }

    Matrix3f trans() {
        Matrix3f res = Matrix3f(0, column, row);

        res.setNum(nums[0][0], 0, 0);
        res.setNum(nums[0][1], 1, 0);
        res.setNum(nums[0][2], 2, 0);
        res.setNum(nums[1][0], 0, 1);
        res.setNum(nums[1][1], 1, 1);
        res.setNum(nums[1][2], 2, 1);
        res.setNum(nums[2][0], 0, 2);
        res.setNum(nums[2][1], 1, 2);
        res.setNum(nums[2][2], 2, 2);


        return res;
    }

    float det2x2(){
        return (nums[0][0]*nums[1][1] - nums[0][1]*nums[1][0]);
    }

    float det3x3() {
        float det = nums[0][0] * (nums[1][1] * nums[2][2] - nums[1][2] * nums[2][1])
                + nums[1][0] * (nums[0][1] * nums[2][2] - nums[0][2] * nums[2][1])
                + nums[2][0] * (nums[0][1] * nums[0][2] - nums[1][1] * nums[1][2]);

        return det;
    }

    Matrix3f inverse2x2(){
        Matrix3f res = Matrix3f(0,2,2);

        float det2x2 = this->det2x2();
        res.setNum( nums[1][1]/det2x2,0,0);
        res.setNum(-nums[0][1]/det2x2,0,1);
        res.setNum(-nums[1][0]/det2x2,1,0);
        res.setNum( nums[0][0]/det2x2,1,1);

        return res;
    }

    Matrix3f inverse3x3(){
        Matrix3f res = Matrix3f(0, 3, 3);

        float det3x3 = this->det3x3();

        res.setNum((nums[1][1] * nums[2][2] - nums[1][2] * nums[2][1]) / det3x3, 0, 0);
        res.setNum((nums[0][2] * nums[2][1] - nums[0][1] * nums[2][2]) / det3x3, 0, 1);
        res.setNum((nums[0][1] * nums[1][2] - nums[0][2] * nums[1][1]) / det3x3, 0, 2);
        res.setNum((nums[1][2] * nums[2][0] - nums[1][0] * nums[2][2]) / det3x3, 1, 0);
        res.setNum((nums[0][0] * nums[2][2] - nums[0][2] * nums[2][0]) / det3x3, 1, 1);
        res.setNum((nums[0][2] * nums[1][0] - nums[0][0] * nums[1][2]) / det3x3, 1, 2);
        res.setNum((nums[1][0] * nums[2][1] - nums[1][1] * nums[2][0]) / det3x3, 2, 0);
        res.setNum((nums[0][1] * nums[2][0] - nums[0][0] * nums[2][1]) / det3x3, 2, 1);
        res.setNum((nums[0][0] * nums[1][1] - nums[0][1] * nums[1][0]) / det3x3, 2, 2);

        return res;
    }

    Matrix3f inverse() {
        if (row == 3 && column == 3) {
            return inverse3x3();
        }else if(row == 2 && column == 2){
            return inverse2x2();
        }else{
            Util::myException("matrix error at inverse\n\r");
            return Matrix3f();
        }


    }

    void print() {
        for (int i = 0; i < row; i++) {
            for (int j = 0; j < column; j++) {
                stdout->putFloat(nums[i][j]);
                stdout->putString("\t");
            }
            stdout->putString("\n\r");
        }
        stdout->putString("\n\r");

    }
};


#endif /* MATRIX3F_HPP_ */
