#include "fixedpoint.h"
#include "in4073.h"

// transform the float number into int32_t to speed up the calculation with float number
int32_t flo2fix(float value)
{
	//float a = 0;
	//scanf("%f", &a);
        float *floatvalue = &value;
        int fix = 0;
        int tmp = 0;
        int exp = 0;
        int tail = 0;
        tmp = *((int*)floatvalue);
        fix = tmp & SIGN_BIT;

        exp = ((tmp & EXP_BIT) >> 23) - 127;
        tail = ((tmp & TAIL_BIT) | 0x00800000); 
        fix = fix | ((tail >> (23-exp)) << 16);

        fix = fix | ((tail & ~(0xffffffff << (23-exp))) >> (7-exp));
        //printf("%x\n",fix);
	return fix;
}

// transform the int16_t number into int32_t since the butterworth needs 14 bits fraction calculation ???
int32_t _16to32(int16_t value)
{
        return value << 14;
}

// transform the int32_t number into int16_t
int16_t _32to16(int32_t value)
{
        return value >> 14;
}

// multiply two int32_t number, so it needs right-shifting by 14 bits to get back to the int32_t
int64_t multiply(int32_t a, int32_t b)
{
        //int32_t result;
        int64_t temp;
        temp = (int64_t)(a * b);
        //result = (int32_t)(temp >> 14);
        return temp;
}