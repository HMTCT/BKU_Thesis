#include "readSerial.h"
#include "kinematics.h"
#include "moving.h"

//−−−−−−−−−−VARIABLES USED FOR THE SERIAL DATA TRANSFER−−−−−−−−−−//
#define MAX_DATA_SIZE   10
bool newCommands = false;                           //new commands available / not available
char DATA[MAX_DATA_SIZE];                           //Data received is stored here
bool newExtracted = false;                          //new extracted data available
const char startChar = '!';                         //Message start
const char endChar = '#';                           //Message end

#define AUTO_MODE     "goauto"
#define MANUAL_MODE   "manual"
#define GO_HOME_CMD   "gohome"

bool isAuto = false; //Auto mode run first

void readSerial(){
    static bool dataTransferring = false;       //true if data transfer in progress
    static byte i = 0;                          //index
    char rc;                                    //Received character

    while(Serial.available() > 0 && newCommands == false && i < MAX_DATA_SIZE)
    {
        rc = Serial.read();                     //Reads one char of the data
        if (rc == startChar){                   //Start data transfer if startChar found
            dataTransferring = true;
        }
        else if (dataTransferring == true)
        {
            if (rc != endChar){                 //Transfer data as long as endChar notfound
                DATA[i] = rc;                   //Save data
                i++;
            }
            else{                               //Stop data transfer if endChar found
                DATA[i]='\0';                   //End the string
                Serial.println("Received data:");
                Serial.println(DATA);
                newCommands = true;             //New data available
                i = 0;                          //Reset the index
                dataTransferring = false;       //Stop data transfer
            }
        }
    }
    
}

void commandToKinematics(float* Xcurr, float* Jcurr){
  if (newCommands == false)
    //do nothing
    return;
  float Xnext[6] = {0, 0, 0, 0, 0, 0}; //next pos value
  float Jnext[6]; //next joints value 
  for (int i = 0; i < 6; ++i){
    if (DATA[i] == POSITIVE_DIRECTION){
      //Rotate positive direction
      Xcurr[i] += 10;
    }
    else if (DATA[i] == NEGATIVE_DIRECTION){
      //Rotate negative direction
      Xcurr[i] -= 10;
    }
  }
  ArrayCopy(Xcurr, Xnext);
  InverseK(Xnext, Jnext);

  // move from Jcurr to Jnext
  goStrightLine(Jcurr, Jnext, 0.25e-4, 0.75e-10, 0.0, 0.0);
  // Now Jurr is Jnext
  ArrayCopy(Jnext, Jcurr);

  printCurPos();
  //Reset flag for new command
  newCommands = false;
}

void manualCommand(){
  if (newCommands == false)
    //do nothing 
    return;
  
  if (strcmp(DATA, GO_HOME_CMD) == 0){
    goHomeFromManual();
     //Reset flag for new command
    newCommands = false;
    return;
  }
  float tmp[6];
  ArrayCopy(currJoint, tmp);  //Store current joints
  for (int i = 0; i < 6; ++i){
    int steps = ANGLE_PER_COMMAND/DL[i];
    if (DATA[i] == POSITIVE_DIRECTION){
      //Rotate positive direction
      //singleJointMove(DIR_PIN[i], HIGH, PUL_PIN[i], steps, 2000, 3, 220);
      currJoint[i] += 10;
    }
    else if (DATA[i] == NEGATIVE_DIRECTION){
      //Rotate negative direction
      //singleJointMove(DIR_PIN[i], LOW, PUL_PIN[i], steps, 2000, 3, 220);
      currJoint[i] -= 10;
    }
  }
  simultaneousMove(tmp, currJoint, 0.25e-4, 0.75e-10, 0.0, 0.0);
  printArray(currJoint, 6);
  //Reset flag for new command
  newCommands = false;
}


void excuteCommand(){
  if (strcmp(DATA, AUTO_MODE) == 0 && !isAuto){
    isAuto = true;
    goHomeFromManual();
    //Reset flag for new command
    newCommands = false;
    return;
  }
  if (strcmp(DATA, MANUAL_MODE) == 0 && isAuto){
    isAuto = false;
    //Update currJoint
    for (int i = 0; i < 6; ++i){
      currJoint[i] = getCurPos(i);
    }
    //Reset flag for new command
    newCommands = false;
    return;
  }
  if (isAuto){
    commandToKinematics(Xcurr, Jcurr);
  }
  else{
    manualCommand();
  }
}
