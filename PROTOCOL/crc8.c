#include <stdio.h>
#include <inttypes.h>
unsigned char crc_high_first(unsigned char *ptr, unsigned char len);

uint8_t ss[2] = {0x71,0xAB};
int main()
{
   printf("CRC : %d\n", crc_high_first(ss,2));
   return 0;
}

unsigned char crc_high_first(unsigned char *ptr, unsigned char len)
{
    unsigned char i; 
    unsigned char crc=0x00;

    while(len--)
    {
        crc ^= *ptr++; 
        for (i=8; i>0; --i)   
        { 
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }

    return (crc); 
}
