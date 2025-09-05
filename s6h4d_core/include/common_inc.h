#ifndef COMMON_INC_H
#define COMMON_INC_H

#include <array>
#include <cstdint>

typedef enum {
    RESET = 0,
    PLAY,
    STOP,
} RunStatus;

// typedef struct {
//    float J1;
//    float J2;
//    float J3;
//    float J4;
//    float J5;
//    float J6;
// } JointAngle;
// typedef float JointAngle[6];
using JointAngle = std::array<float, 6>;


typedef struct {
    float angle;
    float vel;
    float accel;
} JointMsg;

typedef struct {
    uint8_t cmd[4];
    JointMsg joint[6];
} S6h4dTxMsg;

#endif