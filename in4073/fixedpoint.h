#ifndef FIXEDPOINT_H__
#define FIXEDPOINT_H__

#include <inttypes.h>

#define SIGN_BIT 0x80000000
#define EXP_BIT  0x7f800000
#define TAIL_BIT 0x007fffff

int32_t flo2fix(float value);
int32_t _16to32(int16_t value);
int16_t _32to16(int32_t value);
int64_t multiply(int32_t a, int32_t b);

#endif