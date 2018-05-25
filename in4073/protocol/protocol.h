#ifndef PROTOCOL_H__
#define PROTOCOL_H__

#include <stdio.h>
#include <inttypes.h>

/* data framing */
#define HEADER_VALUE							0x80
#define MAX_PAYLOAD								30
#define PACKET_OVERHEAD						5
#define PACKET_ACK_LENGTH   			6
#define PACKET_TELEMETRY_LENGTH		28

/* type of packet */
#define PACKET_GENERAL		10
#define PACKET_ACK			  11
#define PACKET_TELEMETRY	12

/* the length for each type of packet */
#define LENGTH_GENERAL		5
#define LENGTH_ACK				1
#define LENGTH_TELEMETRY	23


#define FLAG_1  1   /* 0000 0001 */		//MESSAGE COMPLETELY RECIEVED
#define FLAG_2  2   /* 0000 0010 */		//MESSAGE WITH CRC ERROR RECEIVED
#define FLAG_3  4   /* 0000 0100 */
#define FLAG_4  8   /* 0000 1000 */
#define FLAG_5  16  /* 0001 0000 */
#define FLAG_6  32  /* 0010 0000 */
#define FLAG_7  64  /* 0100 0000 */
#define FLAG_8  128 /* 1000 0000 */

/* modes we have */
// enum modes
//  {
//  	MODE_SAFE = 	0X00,
//  	MODE_PANIC = 	0X01,
//  	MODE_MANUAL = 	0x02
//  };

/* all kinds of modes */
#define SAFE_MODE		0X00
#define PANIC_MODE		0X01
#define MANUAL_MODE		0x02
#define CALIBRATION_MODE	0x03
#define YAW_CONTROL_MODE	0x04
#define FULL_CONTROL_MODE	0x05

#define P_YAW_UP 			0x01
#define P_YAW_DOWN			0x02
#define P1_ROLL_PITCH_UP	0x04	
#define P1_ROLL_PITCH_DOWN	0x08
#define P2_ROLL_PITCH_UP	0x10
#define P2_ROLL_PITCH_DOWN	0x20

uint8_t flags;

enum msg_status {
	INIT,
	GOT_LEN,
	GOT_ID,
	GOT_P_ADJUST,
	GOT_DATA,
	GOT_CRC,
	GOT_PACKET
};

struct packet{
		enum msg_status status;
		uint8_t header;
		uint8_t length;
		uint8_t packet_id;
		uint8_t p_adjust;
		int8_t data[MAX_PAYLOAD]; // please obey the data sequence which is data[0] = mode, data[1] = lift, data[2] = pitch, data[3] = roll, data[4] = yaw
		uint8_t crc;
		uint8_t i;
		uint8_t crc_fails;
};

struct packet pc_packet;

void protocol_init();
unsigned char crc_high_first(int8_t *ptr, unsigned char len);
void create_packet(uint8_t length, uint8_t packet_id, uint8_t p_adjust, int8_t *data, int8_t *packet);
void create_ack(uint8_t length, uint8_t p_adjust, int8_t data, uint8_t *ack_packet);
void create_telemetry_packet(uint8_t length, uint8_t p_adjust, int8_t *data, int8_t *telemetry_packet);
uint8_t parse_packet(struct packet *rx, uint8_t c);


#endif
