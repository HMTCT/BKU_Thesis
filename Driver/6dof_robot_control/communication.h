#ifndef __COMMUNICATION__H
#define __COMMUNICATION__H

#include "Arduino.h"
#include "global.h"

#define MAX_DATA_SIZE 100

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
  AUTO_MOVE,
  INVALID
};

class SerialCommunication {
  private:
    CONTROL_MODE mode;
    CONTROL_CMD  cmd;
    char DATA[MAX_DATA_SIZE];   //Data received is stored here
    bool new_data;              //Verify new data
  public:
      SerialCommunication();
      void read();
      void validate();
      char* getData();
      void getAxis(float*);
      bool newData();
      CONTROL_CMD getCommand();
};

#endif
