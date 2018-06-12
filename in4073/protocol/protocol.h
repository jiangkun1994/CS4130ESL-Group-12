#ifndef PROTOCOL_H__
#define PROTOCOL_H__

#include <stdio.h>
#include <inttypes.h>

//#define DEBUG_PROTOCOL

/* data framing */
#define HEADER_VALUE							0x80
#define MAX_PAYLOAD								80
#define PACKET_OVERHEAD						4
#define PACKET_ACK_LENGTH   			7
#define PACKET_TELEMETRY_LENGTH		27

/* type of packet */
#define PACKET_GENERAL		10
#define PACKET_ACK			  11
#define PACKET_TELEMETRY	12
#define PACKET_LOG				13

/* the length for each type of packet */
//#define LENGTH_GENERAL		5
#define LENGTH_ACK				2
//#define LENGTH_TELEMETRY	23


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
/* all kinds of modes */
#define SAFE_MODE								0X00
#define PANIC_MODE							0X01
#define MANUAL_MODE							0x02
#define CALIBRATION_MODE				0x03
#define YAW_CONTROL_MODE				0x04
#define FULL_CONTROL_MODE				0x05
#define RAW_MODE					0x06
#define HEIGHT_CONTROL_MODE			0x07
#define HEIGHT_CONTROL_MODE_END	0x0A
#define END_MODE								0x09
#define IDLE_MODE								0x10

#define P_YAW_UP 								0x01
#define P_YAW_DOWN							0x02
#define P1_ROLL_PITCH_UP				0x04
#define P1_ROLL_PITCH_DOWN			0x08
#define P2_ROLL_PITCH_UP				0x10
#define P2_ROLL_PITCH_DOWN			0x20
#define P3_HEIGHT_DOWN					0x15
#define P3_HEIGHT_UP			  		0x16
#define LOGGING_DATA		0x17

uint8_t flags;
extern struct msg_telemetry_template msg_teleTX;
extern struct msg_telemetry_template msg_teleRX;
extern struct msg_pc_template msg_pcTX;
extern struct msg_pc_template *msg_pcRX;

enum msg_status {
	INIT,
	GOT_LEN,
	GOT_ID,
	GOT_DATA,
	GOT_CRC,
	GOT_PACKET,
};

struct packet{
		enum msg_status status;
		uint8_t header;
		uint8_t length;
		uint8_t packet_id;
		int8_t data[MAX_PAYLOAD]; // please obey the data sequence which is data[0] = mode, data[1] = lift, data[2] = pitch, data[3] = roll, data[4] = yaw
		uint8_t crc;
		uint8_t index;
		uint8_t crc_fails;
		uint8_t p_adjust;
		uint8_t logging;
}__attribute__((packed));

struct packet pc_packet;

struct msg_pc_template{
	uint8_t mode;
	int8_t lift;
	int8_t roll;
	int8_t pitch;
	int8_t yaw;
	uint8_t P;
	uint8_t LOGGING;
}__attribute__((packed));

struct msg_telemetry_template{
	uint8_t mode;
	uint8_t lift;
	int8_t roll;
 	int8_t pitch;
 	int8_t yaw;
 	int16_t engine[4];
 	int16_t phi, theta, psi;
  int16_t sp, sq, sr;
  int16_t sax, say, saz;
  uint16_t bat_volt;
  uint8_t P;
  uint8_t P1;
  uint8_t P2;
  uint8_t P3;
  int32_t pressure;
	int32_t temperature;
	uint32_t Time_stamp;
}__attribute__((packed));

void protocol_init();
uint8_t crc_high_first(uint8_t *ptr, unsigned char len);
void create_packet(uint8_t length, uint8_t packet_id, uint8_t *data, uint8_t *packet);
void create_ack(uint8_t length, uint8_t *data, uint8_t *ack_packet);
//void create_telemetry_packet(uint8_t length, int8_t *data, uint8_t *telemetry_packet);
uint8_t parse_packet(struct packet *rx, uint8_t c);


#endif
