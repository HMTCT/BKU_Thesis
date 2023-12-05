#ifndef __MOVING__H
#define __MOVING__H

void goHomeFromManual();
void singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530);
void simultaneousMove(float* Jcurr, float* Jnext, float vel0, float acc0, float velini, float velfin);

#endif