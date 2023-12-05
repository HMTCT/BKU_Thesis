#ifndef __READ_SERIAL__H
#define __READ_SERIAL__H

void readSerial();
void commandToKinematics(float* Xcurr, float* Jcurr);
void manualCommand();
void excuteCommand();

#endif