#ifndef _MRCANDEFS_H_
#define _MRCANDEFS_H_

#define ROSMSG_DRIVE 0
#define ROSMSG_STEERCURVE 1
#define ROSMSG_STEERPARA 2
#define ROSMSG_SPIN 3

#define COMMAND_DRIVE 0
#define COMMAND_STEERCURVE 1
#define COMMAND_STEERPARA 2
#define COMMAND_SPIN 3
#define COMMAND_SPINSTEER 4
// Add start
//Control speed motor drive
#define COMMAND_SPEED_FL 51 //'c'
#define COMMAND_SPEED_FR 52 //'d'
#define COMMAND_SPEED_BL 53 //'e'
#define COMMAND_SPEED_BR 54 //'f'
#define COMMAND_SPEED_LR 61 //'m'

// Control angle motor steer
#define COMMAND_STEER_FL 55 //'g'
#define COMMAND_STEER_FR 56 //'h'
#define COMMAND_STEER_BL 57 //'i'
#define COMMAND_STEER_BR 58 //'j'
#define COMMAND_STEER_FB 59 //'k'
#define COMMAND_CURVE_4W 60 //'l'

#define COMMAND_REAL_TIME_SPEED 71 //'w'
#define COMMAND_STOP_MOTOR 72 //'x'

// Add end
#define COMMAND_CHANGESPEED 7
#define COMMAND_RELEASE 8
#define COMMAND_RESETALARM 9

#define COMMAND_INFO 35     //'S'
#define COMMAND_SETSMODE 38 //'V'
#define COMMAND_SETPMODE 40 //'X'
#define COMMAND_SDRIVE 49   //'a'
#define COMMAND_SSTOP 50    //'b'
#define COMMAND_SSPIN 66    //'r'

#endif
