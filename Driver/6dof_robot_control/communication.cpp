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
  this->new_data = false;
}

void SerialCommunication::read() {
  this->new_data = false;
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
        this->new_data = true;
        i = 0;                     //Reset the index
        dataTransferring = false;  //Stop data transfer
#ifdef DEBUG
        Serial.println("Received data:");
        Serial.println(DATA);
#endif
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
        strcpy(this->DATA, STOP_CMD);
      }
      break;
    case MANUAL:
      if (strcmp(this->DATA, AUTO_CMD) == 0){
        this->cmd = GO_HOME;
        this->mode = AUTO;
        strcpy(this->DATA, STOP_CMD);
      }
      else if (strcmp(this->DATA, GO_HOME_CMD) == 0){
        this->cmd = GO_HOME;
        strcpy(this->DATA, STOP_CMD);
      }
      else if (strcmp(this->DATA, GO_FOLD_CMD) == 0){
        this->cmd = GO_FOLD;
        this->mode = SLEEP;
      }
      else if (strcmp(this->DATA, STOP_CMD) == 0){
        if (this->cmd == MANUAL_MOVE){
          this->cmd = STOP;
        }
        else{
          this->cmd = IDLE;
        }
      }
      else if (this->new_data)
      {
        this->cmd = MANUAL_MOVE;
      }
      else 
      {
        this->cmd = IDLE;
      }
      break;
    case AUTO:
      if (strcmp(this->DATA, MANUAL_CMD) == 0){
        this->mode = MANUAL;
        this->cmd = IDLE;
        strcpy(this->DATA, STOP_CMD);
      }
      else if (this->new_data)
      {
        this->cmd = AUTO_MOVE;
#ifdef DEBUG
        Serial.println("PROCESS MOVING COORDINATES");
#endif
      }
      else 
      {
        this->cmd = IDLE;
      }
      break;
    case SLEEP:
      if (strcmp(DATA, GO_HOME_CMD) == 0){
        this->cmd = WAKEUP;
        this->mode = MANUAL;
        strcpy(this->DATA, STOP_CMD);
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
bool SerialCommunication::newData(){
  return this->new_data;
}
void SerialCommunication::getAxis(float* output){
    String token = "";
    int i = 0;
    for (char c : this->DATA) {
        if (i == 6) break;
        if (c != ':' && c != '\0') {
            token += c;
        } 
        else {
            // Convert String to char array
            char charArray[token.length() + 1]; // +1 for null terminator
            token.toCharArray(charArray, token.length() + 1);
            //Convert to float
            float floatValue = atof(charArray);
            output[i++] = floatValue;
            token = "";
        }
    }
#ifdef DEBUG
    Serial.println("GET AXIS");
    for (int i = 0; i < 6; i++)
    {
      Serial.println(output[i]);
    }
#endif
}
