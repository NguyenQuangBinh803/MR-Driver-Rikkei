#ifndef _MRCAN_H_
#define _MRCAN_H_
#include <Arduino.h>

class MCP_CAN;
class MRCAN
{
  private:
  MCP_CAN *mcp;
  HardwareSerial *ser;
  float MR_L=2;
  float MR_W=1;
  unsigned long int ids[2][4]={{0x00A,0x003,0x004,0x007},{0x001,0x002,0x005,0x006}};
  float spin_ccm;
  float cspind=14.050; //default value for (L,W)=(2,1)
  float cspins=721.75; //default value for (L,W)=(2,1)
  unsigned long int wt=6;
  unsigned long int wtset=50;
  unsigned long int wspin=1500;
  /*
  const int parity_drive[4] = {1,-1,-1,1}; //sign of driving direction
  const int parity_curve[4] = {1,1,-1,-1}; //sign of steering directoin for curve
  const int parity_para[4] = {1,1,1,1}; //sign of steering direction for tlanslation
  const int parity_spins[4] = {1,-1,-1,1}; //sign of sttering direction for spin
  const int parity_spind[4] = {1,1,1,1}; //sign of driving direction for spin
  */
  
  float coef[8]={0x2C00, 11.37, 11.37, cspind, cspins,0,0,0x67};//from {(m),(deg),(deg),(deg),1,,,(km/h)} to hex value of the command.
  const int parity[8][4]={{1,-1,-1,1}, //DRIVE
                          {1,1,-1,-1}, //STEERCURVE
                          {1,1,1,1},   //PARA
                          {1,1,1,1}, //SPINDRIVE
                          {1,-1,1,-1}, //SPINSTEER
                          {},
                          {},
                          {1,1,1,1}};//CHANGESPEED
  
  //[0][][]: drive, [1][][]: steer 
  byte command_setting[2][5][8] ={{{0x00,0xDA,0x00,0x19,0x00,0x00,0x00,0x3F}, //set position mode
                                   {0x00,0xDA,0x00,0x12,0x00,0x00,0xA0,0xA0}, //set acc/dec time to max speed
                                   {0x00,0xDA,0x00,0x14,0x00,0x00,0x00,0x67}, //set max speed
                                   {0x00,0xDA,0x00,0x17,0x00,0x00,0x00,0x5F}, //set control mode to relative
                                   {0x00,0xDA,0x00,0x10,0x00,0x00,0x00,0x1F}},//enable motor
                                  {{0x00,0xDA,0x00,0x19,0x00,0x00,0x00,0x3F}, //set position mode
                                   {0x00,0xDA,0x00,0x12,0x00,0x00,0x01,0x01}, //set acc/dec time to max speed
                                   {0x00,0xDA,0x00,0x14,0x00,0x00,0x00,0x05}, //set max speed
                                   {0x00,0xDA,0x00,0x17,0x00,0x00,0x00,0x4F}, //set control mode to absolute
                                   {0x00,0xDA,0x00,0x10,0x00,0x00,0x00,0x1F}}};//enable motor

  byte command_setting_smode[4][8] = {{0x00,0xDA,0x00,0x10,0x00,0x00,0x00,0x1F}, //release motor
                                      {0x00,0xDA,0x00,0x19,0x00,0x00,0x00,0x2F}, //set speed mode
                                      {0x00,0xDA,0x00,0x13,0x00,0x00,0xA0,0xA0}, //set acc/dec time to max speed
                                      {0x00,0xDA,0x00,0x11,0x00,0x00,0x00,0x00}}; //set max speed
                                      
  byte command_release[8] ={0x00,0xDA,0x00,0x10,0x00,0x00,0x00,0x0F};
  byte command_unrelease[8] ={0x00,0xDA,0x00,0x10,0x00,0x00,0x00,0x1F};
  
  byte command_resetalarm[8] ={0x00,0xDA,0x00,0x15,0x00,0x00,0x00,0x7F};
  
  byte command_sdrive[4][8] = {{0x00,0xDA,0x00,0x11,0x00,0x00,0x00,0x67},
                            {0x00,0xDA,0x00,0x11,0xFF,0xFF,0xFF,0x99},
                            {0x00,0xDA,0x00,0x11,0xFF,0xFF,0xFF,0x99},
                            {0x00,0xDA,0x00,0x11,0x00,0x00,0x00,0x67}};
  byte command_sstop[8]  = {0x00,0xDA,0x00,0x11,0x00,0x00,0x00,0x00};
  
  
  
  float Rcurve=0;
  float Vdrive=1;
  boolean isStraight=true;
  boolean readyMove=false;
  boolean settingChanged=true;
  boolean isSpinState=false;
  boolean isReleased=false;
  boolean isInnerParamChanged=false;
  boolean isSpeedMode=false;
  int kind;
  float param;
  
  
  unsigned long int rxId;
  unsigned char len;
  unsigned char rxBuf[8];


private:
  void sndMsgBuf(int id, int mode, int len, byte cmd[8]);
  void init_size(float L, float W);
  void setMoveCommand(int kind, float param, byte commands[4][8]);
  void setParams(int kind, float param, float *params);
  void setSpeedCommand(int kind,float param, byte commands[4][8]);
  void sendSettingCommand_SpeedMode();
  void MRCAN::sendSettingCommand_PositionMode();
public:
  
  MRCAN(MCP_CAN* mcp, byte CAN_CS);
  MRCAN(MCP_CAN* mcp, byte CAN_CS, float L, float W);
  void To_String(char *msg);
  boolean readCommand(String str);
  void sendSettingCommand();
  void sendMoveCommand();
  void readCANdata(int mcpINT);
  void setWaitTime(int waitTime);
  void setSerial(HardwareSerial* pser);
  byte begin(byte speedset, const byte clockset);
};



#endif
