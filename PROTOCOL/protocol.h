/* Function: header file for protocol.
 * */

#ifndef _PROTOCOL_H
#define _PROTOCOL_H

// mode definition
#define SAFE_MODE 		0x00
#define PANIC_MODE 		0x01
#define MANUAL_MODE 		0x02
#define CALIBRATION_MODE 	0x03
#define YAW_CONTROL_MODE 	0x04
#define FULL_CONTROL_MODE 	0x05
#define RAW_MODE 		0x06
#define HEIGHT_CONTROL_MODE 	0x07
#define WIRELESS_MODE 		0X08


typedef struct DataFrame{
	char header;
	char length;
	char mode;
	char lift;
	char yaw;
	char roll;
	char pitch;
	char CRCvalue;
} packet;

#endif
