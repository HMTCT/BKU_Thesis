#ifndef __COMMUNICATION__H
#define __COMMUNICATION__H

#include "Arduino.h"

#define MAX_DATA_SIZE 10

enum CONTROL_MODE{
  INIT,
  SLEEP,
  AUTO,
  MANUAL,
};
enum CONTROL_CMD{
  IDLE,
  WAKEUP,
  GO_HOME,
  GO_FOLD,
  STOP,
  MANUAL_MOVE,
  INVALID
};

class SerialCommunication {
  private:
    CONTROL_MODE mode;
    CONTROL_CMD  cmd;
    char DATA[MAX_DATA_SIZE];   //Data received is stored here
  public:
      SerialCommunication();
      void read();
      void validate();
      char* getData();
      CONTROL_CMD getCommand();
};

#endif