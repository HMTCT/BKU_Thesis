#include "communication.h"

//−−−−−−−−−−VARIABLES USED FOR THE SERIAL DATA TRANSFER−−−−−−−−−−//

#define AUTO_CMD    "goauto"
#define MANUAL_CMD  "manual"
#define GO_HOME_CMD "gohome"
#define STOP_CMD    "000000"
#define INITIAL_CMD "Initon"
#define GO_FOLD_CMD "gofold"
#define startChar   '!'         //Message start
#define endChar     '#'         //Message end

SerialCommunication::SerialCommunication(){
  this->mode = INIT;
  this->cmd = INVALID;
  memset(this->DATA, 0, MAX_DATA_SIZE * sizeof(char));
}

void SerialCommunication::read() {
  static bool dataTransferring = false;  //true if data transfer in progress
  static byte i = 0;                     //index
  char rc;                               //Received character
  while (Serial.available() > 0 && i < MAX_DATA_SIZE) {
    rc = Serial.read();     //Reads one char of the data
    if (rc == startChar) {  //Start data transfer if startChar found
      dataTransferring = true;
    } 
    else if (dataTransferring) {
      if (rc != endChar) {  //Transfer data as long as endChar notfound
        DATA[i] = rc;       //Save data
        ++i;
      } 
      else {                //Stop data transfer if endChar found
        DATA[i] = '\0';     //End the string
        // Serial.println("Received data:");
        i = 0;                     //Reset the index
        dataTransferring = false;  //Stop data transfer
        Serial.println(DATA);
      }
    }
  }
}

void SerialCommunication::validate(){
  switch (this->mode){
    case INIT:
      if (strcmp(this->DATA, INITIAL_CMD) == 0){
        this->cmd = WAKEUP;
        this->mode = MANUAL;
      }
      break;
    case MANUAL:
      if (strcmp(this->DATA, AUTO_CMD) == 0){
        this->cmd = GO_HOME;
        this->mode = AUTO;
      }
      else if (strcmp(this->DATA, STOP_CMD) == 0){
        if (this->cmd == MANUAL_MOVE){
          this->cmd = STOP;
        }
        else{
          this->cmd = IDLE;
        }
      }
      else if (strcmp(this->DATA, GO_HOME_CMD) == 0){
        this->cmd = GO_HOME;
      }
      else if (strcmp(this->DATA, GO_FOLD_CMD) == 0){
        this->cmd = GO_FOLD;
        this->mode = SLEEP;
      }
      else{
        this->cmd = MANUAL_MOVE;
      }
      break;
    case AUTO:
      if (strcmp(this->DATA, MANUAL_CMD) == 0){
        this->mode = MANUAL;
      }
      break;
    case SLEEP:
      if (strcmp(DATA, GO_HOME_CMD) == 0){
        this->cmd = WAKEUP;
        this->mode = MANUAL;
      }
      else{
        this->cmd = IDLE;
      }
      break;
    default:
      this->cmd = INVALID;
      this->mode = SLEEP;
      break;
  }
}

CONTROL_CMD SerialCommunication::getCommand(){
  return this->cmd;
}

char* SerialCommunication::getData(){
  return this->DATA;
}