//Kinematics library
#ifndef __KINEMATICS__H
#define __KINEMATICS__H

#include "Arduino.h"

void goStrightLine(float* xfi, float* xff, float vel0, float acc0, float velini, float velfin);
void goTrajectory(float* Jf);
void InverseK(float* Xik, float* Jik);
void ForwardK(float* Jfk, float* Xfk);
void invtran(float* Titi, float* Titf);
void tran2pos(float* Ttp, float* Xtp);
void pos2tran(float* Xpt, float* Tpt);
void DH1line(float thetadh, float alfadh, float rdh, float ddh, float* Tdh);
void MatrixPrint(float* A, int m, int n, String label);
void MatrixCopy(float* A, int n, int m, float* B);
void MatrixMultiply(float* A, float* B, int m, int p, int n, float* C);
void MatrixAdd(float* A, float* B, int m, int n, float* C);
void MatrixSubtract(float* A, float* B, int m, int n, float* C);
void MatrixTranspose(float* A, int m, int n, float* C);
void MatrixScale(float* A, int m, int n, float k);
void setCurPos(double pos1, double pos2, double pos3, double pos4, double pos5, double pos6);
float getCurPos(uint8_t i);
void printCurPos();

#endif