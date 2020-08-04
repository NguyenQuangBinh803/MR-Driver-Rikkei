
#include "MRCAN.h"
#include "MRCAN_defs.h"
#include <stdio.h>
#include <mcp_can.h>
#include <String.h>
#include <math.h>

//private functions

//#define DEBUG_MODE


void MRCAN::sndMsgBuf(int id, int mode,int len, byte cmd[8]){
  #ifndef DEBUG_MODE
    mcp->sendMsgBuf(id,mode,len,cmd);
  #else
    char msgString[128];
    sprintf(msgString, "%lu Standard ID: 0x%.3lX       DLC: %1d  Data:",millis(), (unsigned long int)id, len);
    ser->print(msgString);
    for(byte i = 0; i<len; i++){
      sprintf(msgString, " 0x%.2X", cmd[i]);
      ser->print(msgString);
    }
    ser->println();
  #endif
}
        

void MRCAN::init_size(float L, float W){
  MR_L=L; MR_W=W;
  spin_ccm=M_PI*sqrt(MR_L*MR_L+MR_W*MR_W);
  cspind = 0x2460/(360/spin_ccm);
  cspins = 0x1000*atan(MR_L/MR_W)/2/M_PI;
  coef[3]=cspind;
  coef[4]=cspins;
}

void MRCAN::setParams(int kind, float param, float *params){
  float L;float W; float R;float theta1;float theta2;float r1;float r2;
  switch(kind){
    case COMMAND_DRIVE:
      L=MR_L;
      W=MR_W;
      R=Rcurve;
      if(!isStraight && R!=0){
        r1=sqrt(4*R*R+4*R*W+W*W+L*L)/2/R;
        r2=sqrt(4*R*R-4*R*W+W*W+L*L)/2/R;
      }else if(isStraight){
        r1=1;
        r2=1;
      }else{
        r1=0;
        r2=0;
      }
      params[0]=param*r1;
      params[1]=param*r2;
      params[2]=param*r1;
      params[3]=param*r2;
    break;
    
    case COMMAND_STEERCURVE:
    
      L=MR_L;
      W=MR_W;
      R=param;
      if(abs(2*R)>W){
        theta1=180/M_PI*atan(L/(2*R-W));
        theta2=180/M_PI*atan(L/(2*R+W));
      } else if(abs(2*R)==W){
        theta1=(R>0)?90:180/M_PI*atan(L/(2*R-W));
        theta2=(R<0)?(-90):180/M_PI*atan(L/(2*R+W));
      } else{
        theta1=180/M_PI*atan(L/(2*R-W))+((R>0)?180:0);
        theta2=180/M_PI*atan(L/(2*R+W))-((R<0)?180:0);
      }

      params[0]=theta1;
      params[1]=theta2;
      params[2]=theta1;
      params[3]=theta2;
      Rcurve=R;
      isStraight=false;
    break;
    
    case COMMAND_STEERPARA:
      for(int i=0;i<4;i++){
        params[i]=param;
      }
      Rcurve=0;
      isStraight=true;
    break;
    case COMMAND_CHANGESPEED:
      L=MR_L;
      W=MR_W;
      R=Rcurve;
      if(!isStraight && R!=0){
        r1=sqrt(4*R*R+4*R*W+W*W+L*L)/2/R;
        r2=sqrt(4*R*R-4*R*W+W*W+L*L)/2/R;
      }else if(isStraight||isSpinState){
        r1=1;
        r2=1;
      }else{
        r1=0;
        r2=0;
      }
      params[0]=param*r1;
      params[1]=param*r2;
      params[2]=param*r1;
      params[3]=param*r2;
    break;
    

    default:
      for(int i=0;i<4;i++){
        params[i]=param;
      }
      Rcurve=0;
      isStraight=false;
    break;
    
  }
}

void MRCAN::setMoveCommand(int kind, float param, byte commands[4][8]){
  float params[4];
  setParams(kind,param,params);
  
  
  for(int id=0;id<4;id++){
    commands[id][0]=0x00;
    commands[id][1]=0xDA;
    commands[id][2]=0x00;
    commands[id][3]=0x16;
    signed long int x=(signed long int)(coef[kind]*parity[kind][id]*params[id]);
    if(isSpeedMode && kind==COMMAND_CHANGESPEED){
      x=x*parity[COMMAND_DRIVE][id];
    }
    for(int i=4;i<8;i++){
      commands[id][i]=((x>>(56-8*i)) & 0xFF);
    }
  }
}

void MRCAN::setSpeedCommand(int kind, float param, byte commands[4][8]){
  setMoveCommand(kind, param, commands);
  byte reg=0x14;
  if(isSpeedMode){
    reg=0x11;
  }
  for(int id=0;id<4;id++){
    commands[id][3]=reg;
  }
}


//public functions
MRCAN::MRCAN(MCP_CAN* pmcp, byte mcpCS){
  mcp=pmcp;
  mcp->init_CS(mcpCS);
  init_size(2,1);
}

MRCAN::MRCAN(MCP_CAN* pmcp, byte mcpCS, float L, float W){
  mcp=pmcp;
  mcp->init_CS(mcpCS);
  init_size(L,W);
}

void MRCAN::To_String(char *msg){
  char str_L[7];
  char str_W[7];
  char str_Vdrive[7];
  char str_Rcurve[7];
  char str_cspind[7];
  dtostrf(MR_L, 4, 3, str_L); //No %f designator in Arduino. Use dtostrf() and %s designator instead.
  dtostrf(MR_W, 4, 3, str_W);
  dtostrf(Vdrive, 4, 3, str_Vdrive);
  dtostrf(Rcurve, 4, 3, str_Rcurve);
  dtostrf(cspind, 4, 3, str_cspind);
  
  sprintf(msg, "Length: %s, Width: %s, Max driving speed: %s, curving radius: %s, spind coefficient: %s", str_L, str_W, str_Vdrive, str_Rcurve,str_cspind);
}

boolean MRCAN::readCommand(String str){
  kind=(int)(str.charAt(0)-'0');
  param=(str.substring(2)).toFloat();
  readyMove=true;
  return (kind >= 0 && kind < 4) || (kind >= 7 && kind < 10) || kind == 66 ||
          kind ==35 || kind == 49 || kind == 50 || kind ==38 || kind == 40;
}

void MRCAN::sendSettingCommand(){
  if (!readyMove) return; //exit if no new command received.
  if (!settingChanged) return; //exit if no settings changed from the sent setting.
  for (int groupId=0;groupId<2;groupId++){
    for (int id=0;id<4;id++){
      for(int icom=0;icom<5;icom++){
        sndMsgBuf(ids[groupId][id],0,8,command_setting[groupId][icom]);
        delay(wtset);
      }
    }
  }
  settingChanged=false;
  isSpeedMode=false;
}
void MRCAN::sendSettingCommand_PositionMode(){
  for (int groupId=0;groupId<2;groupId++){
    for (int id=0;id<4;id++){
      for(int icom=0;icom<5;icom++){
        sndMsgBuf(ids[groupId][id],0,8,command_setting[groupId][icom]);
        delay(wtset);
      }
    }
  }
  settingChanged=false;
  isSpeedMode=false;
}


void MRCAN::sendSettingCommand_SpeedMode(){
  int groupId=0;
  for (int id=0;id<4;id++){
    for(int icom=0;icom<4;icom++){
      sndMsgBuf(ids[groupId][id],0,8,command_setting_smode[icom]);
      delay(wtset);
    }
  }
  isSpeedMode=true;
  settingChanged=false;
}


void MRCAN::sendMoveCommand(){
  if (!readyMove) return; //exit if no new command reeived.
  if (kind==(int)('S'-'0')){
    char msgString[128];
    To_String(msgString);
    ser->println(msgString);
    return;
  }
  //set commands
  byte moveCommands[4][8];
  int groupId;
  ser->println(kind);
  ser->println(param);
  
  if(kind==COMMAND_SETPMODE){
    sendSettingCommand_PositionMode();
    return;
  }else if(kind==COMMAND_SETSMODE){
    sendSettingCommand_SpeedMode();
    return;
  }
  if(kind==COMMAND_SDRIVE){
    if(!isSpeedMode){
      sendSettingCommand_SpeedMode();
      delay(wtset);    
    }
    int groupId=0;
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,command_sdrive[id]);
      delay(wt);
    }
  return;
  }
  if(kind==COMMAND_SSTOP){
    if(isSpeedMode){
      int groupId=0;
      for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,command_sstop);
      delay(wt);
    }
    }
    return;
  }
  if(kind==COMMAND_SSPIN){
    if(isSpinState){
      if(!isSpeedMode){
        sendSettingCommand_SpeedMode();
        delay(wtset);    
      }
      int groupId=0;
      for(int id=0;id<4;id++){
        sndMsgBuf(ids[groupId][id],0,8,command_sdrive[0]); //parity {1,1,1,1}
        delay(wt);
      }
    }
    return;
  }
  
  
  if(kind==COMMAND_DRIVE && isSpeedMode){
    sendSettingCommand_PositionMode();
    delay(wtset);
  }
  
  
  //releasing drive motors
  if(kind==COMMAND_RELEASE){
    groupId=0;
    if(param==1) groupId=1;
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,command_release);
      delay(wt);
    }
  isReleased=true;
  return;
  }else if(isReleased){
    groupId=0;
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,command_unrelease);
      delay(wt);
    }
    isReleased=false;
  }
  
  //resetting alarms
  if(kind==COMMAND_RESETALARM){
    for(int gId=0;gId<2;gId++){
      for(int id=0;id<4;id++){
        sndMsgBuf(ids[gId][id],0,8,command_resetalarm);
        delay(wt);
      }
    }
  return;
  }
  
  //change driving speed
  if(kind==COMMAND_CHANGESPEED){
    Vdrive=param;
    setSpeedCommand(kind, param, moveCommands);
    groupId=0;
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,moveCommands[id]);
      delay(wt);
      if(isSpeedMode){
        for(int i=4;i<8;i++){
          command_sdrive[id][i]=moveCommands[id][i];
        }
      }else{
        for(int i=4;i<8;i++){
          command_setting[groupId][2][i]=moveCommands[id][i];
        }
      }
    }
    return;
  }

  //change speed drive motors
  if(kind==COMMAND_DRIVE){
    setSpeedCommand(COMMAND_CHANGESPEED, Vdrive, moveCommands);
    groupId=0;
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,moveCommands[id]);
      delay(wt);
      for(int i=4;i<8;i++){
        command_setting[groupId][2][i]=moveCommands[id][i];
      }
    }
  }

  //extra commands in case of spinstate
  if(kind != COMMAND_SPIN && isSpinState){
    isSpinState=false;
    if(kind==COMMAND_DRIVE){
      groupId=1;
    setMoveCommand(COMMAND_STEERPARA, 0, moveCommands);
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,moveCommands[id]);
      delay(wt);
    }
    delay(wspin);

    }
  }
  
  //extra commands for spin
  if(kind==COMMAND_SPIN){
    isSpinState=true;
     
    //spinsteer
    groupId=1;
    setMoveCommand(COMMAND_SPINSTEER, 1, moveCommands);
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,moveCommands[id]);
      delay(wt);
    }
    
    
    //reset drive speed
    groupId=0;
    Rcurve=0;
    setSpeedCommand(COMMAND_CHANGESPEED, Vdrive, moveCommands);
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,moveCommands[id]);
      delay(wt);
      for(int i=4;i<8;i++){
        command_setting[groupId][2][i]=moveCommands[id][i];
      }
    }
    delay(wspin);
  }

  setMoveCommand(kind, param, moveCommands);
  groupId=(kind==COMMAND_DRIVE ||kind == COMMAND_SPIN)?0:1;
  for(int id=0;id<4;id++){
    sndMsgBuf(ids[groupId][id],0,8,moveCommands[id]);
    delay(wt);
  }
  
}

void MRCAN::readCANdata(int mcpINT){ //read CAN data and send them to serial
  char msgString[128];
  if(!digitalRead(mcpINT))                          // If CAN0_INT pin is low, read receive buffer
  {
    mcp->readMsgBufID(&rxId, &len, rxBuf);              // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)             // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Time: %lu, Extended ID: 0x%.8lX,  DLC: %1d  Data:",millis() , (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "%lu Standard ID: 0x%.3lX       DLC: %1d  Data:",millis(), rxId, len);
  
    ser->print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){            // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      ser->print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        ser->print(msgString);
      }
    }
        
    ser->println();
  }
}

void MRCAN::setSerial(HardwareSerial* pser){
  ser=pser; 
}

byte MRCAN::begin(byte speedset, const byte clockset){
 return mcp->begin(speedset, clockset);
 kind=-1;
}
