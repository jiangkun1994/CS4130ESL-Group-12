#include "fixedpoint.h"
#include "in4073.h"


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

int32_t _16to32(int16_t value)
{
        return value << 16;
}

int16_t _32to16(int32_t value)
{
        return value >> 16;
}

int32_t multiply(int32_t a, int32_t b)
{
        int32_t result;
        int64_t temp;
        temp = (int64_t)a * b;
        result = (int32_t)(temp >> 16);
        return result;
}