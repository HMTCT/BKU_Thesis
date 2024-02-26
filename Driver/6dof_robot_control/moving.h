#ifndef __MOVING__H
#define __MOVING__H

#include "Arduino.h"
#include "communication.h"

class ArmMoving
{
private:
  CONTROL_CMD cmd;
  SerialCommunication listener;
  float currJoint[6]; // current joints angle
  float buffer[6];

public:
  ArmMoving();
  void wakeUp();
  void goHomeFromManual();
  void goFoldFromManual();
  void singleJointMove(uint8_t DIR_PIN, uint8_t DIR, uint8_t PUL_PIN, int totSteps, int delValue = 4000, int incValue = 7, int accRate = 530);
  void manualControl(char *DATA, float vel0, float acc0, float velini, float velfin);
  void listen();
  void move();
};

#endif