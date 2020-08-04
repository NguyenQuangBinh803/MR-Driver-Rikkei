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
#include <MsTimer2.h>


#define mcpINT 2
#define interval 5

#define CS_UNO 10 //for UNO and NANO
#define CS_MEGA 53//for MEGA and DUE
#define CS CS_UNO

//MODE_NORMAL
//MODE_LOOPBACK //for debug
#define mcpMode MODE_NORMAL

//#define DEBUG_MODE //for debug without MCP2515 module





char msg[128];
int i=0;
MCP_CAN mcp(0);
MRCAN mrcan(&mcp, CS, 0.4975, 0.295);



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
  if(Serial.available()){
    if(mrcan.readCommand(Serial.readStringUntil(';'))){
      mrcan.sendSettingCommand();
      mrcan.sendMoveCommand();
    }
  }
}
