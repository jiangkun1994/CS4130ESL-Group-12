#include <stdio.h>
#include <inttypes.h>

/*#define uint8_t unsigned char
#define uint16_t unsigned short 
#define uint32_t unsigned int
*/

uint8_t ss = 0x71;
uint16_t Cal_CRC16(uint8_t* data, uint32_t size);
int main()
{
	printf("0x%x\n", Cal_CRC16(&ss, 1));
	return 0;
}

uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte) 
{
	uint16_t i;
	crcIn = crcIn ^ ((uint16_t)byte << 8);

    	for ( i = 0; i < 8; i++)
    	{
        	if ((crcIn & 0x8000) == 0x8000)
            	crcIn = (crcIn << 1) ^ 0x1021;
        	else
            	crcIn = crcIn << 1;
    	}
 
    	return crcIn & 0xffff;
}

uint16_t Cal_CRC16(uint8_t *data, uint32_t size)
{
	uint16_t crc = 0;
	uint32_t i;

	for (i = 0; i < size; i++)
	{
		crc = UpdateCRC16(crc, data[i]);
	}
	return crc & 0xffffu;
}


