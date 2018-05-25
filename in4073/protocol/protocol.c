#include "protocol.h"

void protocol_init(){
    flags = 0;
}

unsigned char crc_high_first(int8_t *ptr, unsigned char len)
{
    uint8_t i;
    uint8_t crc=0x00;

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

void create_packet(uint8_t length, uint8_t packet_id, uint8_t p_adjust, int8_t *data, int8_t *packet){

    uint8_t i = 0;

    packet[0] = HEADER_VALUE;
    packet[1] = length;
    packet[2] = packet_id;
    packet[3] = p_adjust;

    while(i < length){
        packet[i+4] = data[i];
        i++;
    }
    packet[i+4] = crc_high_first(data, length);

}

void create_ack(uint8_t length, uint8_t p_adjust, int8_t data, uint8_t *ack_packet){
    ack_packet[0] = HEADER_VALUE;
    ack_packet[1] = length;
    ack_packet[2] = PACKET_ACK; //packet id
    ack_packet[3] = p_adjust;
    ack_packet[4] = data;       //ACK or not
    ack_packet[5] = crc_high_first(&data, length);
}

void create_telemetry_packet(uint8_t length, uint8_t p_adjust, int8_t *data, int8_t *telemetry_packet){
    int i = 0;
    telemetry_packet[0] = HEADER_VALUE;
    telemetry_packet[1] = length;
    telemetry_packet[2] = PACKET_TELEMETRY;
    telemetry_packet[3] = p_adjust;
    while(i < length){
        telemetry_packet[i+4] = data[i];
        i++;
    }
    telemetry_packet[i+4] = crc_high_first(data, length);
}

uint8_t parse_packet(struct packet *rx, uint8_t c){
    uint8_t correct = 1;
    switch (rx->status) {
        case INIT:
            if (c == HEADER_VALUE){
                rx->i = 0;
                rx->header = c;
                //printf("HEADER = %d\n", rx->header);
                rx->status++;
            }
            break;
        case GOT_LEN:
            rx->length = c;
            //printf("LENGTH = %d\n", rx->length);
            rx->status++;
            break;
        case GOT_ID:
            rx->packet_id = c;
            //printf("PACKET_ID = %d\n", rx->packet_id);
            rx->status++;
            break;
        case GOT_P_ADJUST:
            rx->p_adjust = c;
            rx->status++;
            break;
        case GOT_DATA:
            rx->data[rx->i] = c;
            //printf("DATA[%d] = %d\n", rx->i,rx->data[rx->i]);
            if(rx->i >= (rx->length-1)){
                rx->status++;
            }
            rx->i++;
            break;
        case GOT_CRC:
            rx->crc = c;
            //printf("CRC RECEIVED=%d\n", rx->crc);
            rx->status++;
            //break;
        case GOT_PACKET:
            if (crc_high_first(rx->data, rx->length) != rx->crc){
                rx->crc_fails++;
                correct = 0;
                flags |= FLAG_2;
                printf("CRC FAIL\n");
            }
            flags |= FLAG_1;
            rx->status = INIT;
            break;

        default:
            rx->status = INIT;
            break;
    }
    return correct;
}
