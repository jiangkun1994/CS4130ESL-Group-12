#ifndef FIXED_H__
#define FIXED_H__

#define SIGN_BIT 0x80000000
#define EXP_BIT  0x7f800000
#define TAIL_BIT 0x007fffff

int32_t flo2fix(float value);
int32_t _16to32(int16_t value);
int16_t _32to16(int32_t value);
int32_t multiply(int32_t a, int32_t b);

#endif