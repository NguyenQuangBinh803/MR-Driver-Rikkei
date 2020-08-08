/*
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 * 
 */





#include <mcp_can.h>
#include "MRCAN.h"
//#include <MsTimer2.h>


#define mcpINT 2
#define interval 5

#define CS_UNO 10 //for UNO and NANO
#define CS_MEGA 53//for MEGA and DUE
#define CS CS_MEGA

//MODE_NORMAL
//MODE_LOOPBACK //for debug
#define mcpMode MODE_NORMAL

//#define DEBUG_MODE //for debug without MCP2515 module





char msg[128];
int i=0;
MCP_CAN mcp(0);
//Edit start
//MRCAN mrcan(&mcp, CS, 0.4975, 0.295);
MRCAN mrcan(&mcp, CS, 0.48, 0.30);
//Edit end
// Add start
boolean flagStopBumper = false, isWaitingSpeedData = false;
byte speeds[4]={0x00,0x00,0x00,0x00};
int count_speeds=0;
// Add end
unsigned long timeLine;



void flash(){
  //
  mrcan.readCANdata(mcpINT);
} 

void setup(){
  Serial.begin(115200);
  mrcan.setSerial(&Serial);
  #ifndef DEBUG_MODE
    if(mcp.begin(CAN_500KBPS, MCP_8MHz) == CAN_OK){
      Serial.println("Initialization success");
    } else{
      Serial.println("Error in initializing...");
    }
    mcp.setMode(mcpMode);
    pinMode(mcpINT,INPUT);

//    MsTimer2::set(interval, flash);
//    MsTimer2::start();
  #endif
  mrcan.sendSettingCommand();

}

void loop(){
  if(mrcan.readyMove){
    if(isWaitingSpeedData){
      if((unsigned long) (millis() - timeLine) > 75){
        isWaitingSpeedData=false;
        sendSpeedToSerial(speeds);
        timeLine = millis();
      }
      if(mrcan.readSpeedMotor(mcpINT, speeds)){
        count_speeds++;
        if(count_speeds==2){
          isWaitingSpeedData=false;
          sendSpeedToSerial(speeds);
          timeLine = millis();
        }
      }
    }else{
      if((unsigned long) (millis() - timeLine) > 75){
        getSpeedMotor();
        timeLine = millis();
        count_speeds=0;
        isWaitingSpeedData=true;
        speeds[0]=0xFF;
        speeds[1]=0x00;
        speeds[2]=0xFF;
        speeds[3]=0x00;
      }
    }
  }
  // Add end
  if(Serial.available()){
    if(mrcan.readCommand(Serial.readStringUntil(';'))){
      mrcan.sendSettingCommand();
      mrcan.sendMoveCommand();
    }
  }
}
// Add start
void getSpeedMotor(){
  mrcan.readCommand("w;");
  mrcan.sendMoveCommand();
}

void sendSpeedToSerial(byte speeds[4]){
    Serial.write(255);
    Serial.write(254);
    Serial.write(speeds, 4);
}
// Add end
