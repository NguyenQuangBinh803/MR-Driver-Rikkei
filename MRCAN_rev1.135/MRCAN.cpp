
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
    mcp->sendMsgBuf(id,mode,len,cmd);
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
        // Edit start
        r1=sqrt(4*R*R+4*R*W+W*W+L*L)/2/abs(R);
        r2=sqrt(4*R*R-4*R*W+W*W+L*L)/2/abs(R);
        // Edit end
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
        // Edit start
        r1=sqrt(4*R*R+4*R*W+W*W+L*L)/2/abs(R);
        r2=sqrt(4*R*R-4*R*W+W*W+L*L)/2/abs(R);
        // Edit end
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
  if(kind==COMMAND_STEER_FB||kind==COMMAND_SPEED_LR){
    char charBuf[50];
    str.substring(2).toCharArray(charBuf, 50);
    char* strSlipt = strtok(charBuf, ",");
    param=atof(strSlipt);
    strSlipt=strtok(NULL, ",");
    param2=atof(strSlipt);
  }else{
    param=(str.substring(2)).toFloat();
    param2=0.0;
  }
  readyMove=true;
  // Edit start
  return (kind >= 0 && kind < 4) || (kind >= 7 && kind < 10) || kind == 66 ||
          kind ==35 || kind == 49 || kind == 50 || kind ==38 || kind == 40 || kind == 5 || (kind >= 51 && kind <= 61 || kind == 71 || kind == 72);
  // Edit end
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
  #ifdef DEBUG_MODE
  ser->println(kind);
  ser->println(param);
  #endif

  // Add start
  if(kind==COMMAND_REAL_TIME_SPEED){
    byte speeds[4]={0x00,0x00,0x00,0x00};
    sndMsgBuf(0x007,0,8,command_read_speed);
    sndMsgBuf(0x004,0,8,command_read_speed);
    return;
  }

  if(kind >= 55 && kind <= 59){
    setMoveCommand(COMMAND_STEERPARA, param, moveCommands);
    switch (kind)
    {
      case COMMAND_STEER_FL:
        sndMsgBuf(ids[1][2],0,8,moveCommands[2]);
        break;
      case COMMAND_STEER_FR:
        sndMsgBuf(ids[1][3],0,8,moveCommands[3]);
        break;
      case COMMAND_STEER_BL:
        sndMsgBuf(ids[1][1],0,8,moveCommands[1]);
        break;
      case COMMAND_STEER_BR:
        sndMsgBuf(ids[1][0],0,8,moveCommands[0]);
        break;
      case COMMAND_STEER_FB:
        #ifdef DEBUG_MODE
        ser->println(param2);
        #endif
        //Control front steer motor
        sndMsgBuf(ids[1][2],0,8,moveCommands[2]);
        sndMsgBuf(ids[1][3],0,8,moveCommands[3]);
        //Control back steer motor
        setMoveCommand(COMMAND_STEERPARA, param2, moveCommands);
        sndMsgBuf(ids[1][1],0,8,moveCommands[1]);
        sndMsgBuf(ids[1][0],0,8,moveCommands[0]);
        break;
    }
    return;
  }

  if(kind==COMMAND_CURVE_4W){
    if(param > 38.0 || param < -38.0 || param == 0) return;
    param = (MR_L/tan(abs(param) * M_PI/180) - MR_W)/2 *((param>0)?1:-1);
    #ifdef DEBUG_MODE
    ser->print("R calculator: ");
    ser->println(param);
    #endif
    kind=COMMAND_STEERCURVE;
  }

  if(kind==COMMAND_STOP_MOTOR){
    groupId=0;
    for(int id=0;id<4;id++){
      sndMsgBuf(ids[groupId][id],0,8,command_stop);
      delay(wt);
    }
    return;
  }

  // Add end
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
  // Edit start
  if(kind==COMMAND_CHANGESPEED || (kind >= 51 && kind <= 54)|| kind==COMMAND_SPEED_LR){
    Vdrive=param;
    
    int kindTemp=kind;
    kind=COMMAND_CHANGESPEED;
    setSpeedCommand(kind, param, moveCommands);
    
    /*
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
    */
    switch (kindTemp)
    {
      case COMMAND_SPEED_FL:
        sendSpeedCommand(2, moveCommands);
        break;
      case COMMAND_SPEED_FR:
        sendSpeedCommand(3, moveCommands);
        break;
      case COMMAND_SPEED_BL:
        sendSpeedCommand(1, moveCommands);
        break;
      case COMMAND_SPEED_BR:
        sendSpeedCommand(0, moveCommands);
        break;
      case COMMAND_CHANGESPEED:
        sendSpeedCommand(-1, moveCommands);
        break;
      case COMMAND_SPEED_LR:
        if(param<param2){
          Vdrive=param2;
        }
        #ifdef DEBUG_MODE
        ser->println(param2);
        #endif
        //Control left motor speed
        sendSpeedCommand(1, moveCommands);
        sendSpeedCommand(2, moveCommands);
        //Control right motor speed 
        setSpeedCommand(COMMAND_CHANGESPEED, param2, moveCommands);
        sendSpeedCommand(0, moveCommands);
        sendSpeedCommand(3, moveCommands);
        break;
    }
    //Edit end
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
//Add start
boolean MRCAN::sendCANsuccess(int mcpINT){ //read CAN data and send them to serial
  if(!digitalRead(mcpINT))                          // If CAN0_INT pin is low, read receive buffer
  {
    mcp->readMsgBufID(&rxId, &len, rxBuf);
    if(len ==8){
      return true;  
    }else{
      return false;
    }
  }
}


boolean MRCAN::readSpeedMotor(int mcpINT, byte speeds[4]){ //read CAN data and send them to serial
  char msgString[128];
  if(!digitalRead(mcpINT))                          // If CAN0_INT pin is low, read receive buffer
  {
    mcp->readMsgBufID(&rxId, &len, rxBuf); 
    #ifdef DEBUG_MODE
    ser->println("Message receive");             // Read data: len = data length, buf = data byte(s)
    //Del
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
    #endif
    if(rxId==0x004 && rxBuf[1]==0xDB && rxBuf[3]==0xE4){
      speeds[0]=rxBuf[6];
      speeds[1]=rxBuf[7];
      return true;
    }
    if(rxId==0x007 && rxBuf[1]==0xDB && rxBuf[3]==0xE4){
      speeds[2]=rxBuf[6];
      speeds[3]=rxBuf[7];
      return true;
    }
    
    return false;
  }
}


void MRCAN::sendSpeedCommand(int id, byte commands[4][8]){
  //If id=-1, change speed 4 wheel drive
  int endloop;
  if(id==-1){
    id=0;
    endloop=4;
  }else{
    endloop=id+1;
  }
  for(id;id<endloop;id++){
      sndMsgBuf(ids[0][id],0,8,commands[id]);
      if(isSpeedMode){
        for(int i=4;i<8;i++){
          command_sdrive[id][i]=commands[id][i];
        }
      }else{
        for(int i=4;i<8;i++){
          command_setting[0][2][i]=commands[id][i];
        }
      }
    }
}
//Add end
