/* All functions: Randy Prozée */
#include "protocol.h"

// Local functions
uint8_t crc_high_first(uint8_t *ptr, unsigned char len);

void protocol_init(){
    flags = 0;
}

uint8_t crc_high_first(uint8_t *ptr, unsigned char len)
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
    crc = crc & 0x7F;
    return (crc);
}

void create_packet(uint8_t length, uint8_t packet_id, uint8_t *data, uint8_t *packet){

    uint8_t i = 0;

    packet[0] = HEADER_VALUE;
    packet[1] = length;
    packet[2] = packet_id;

    while(i < length){
        packet[i+3] = data[i];
        i++;
    }
    packet[i+3] = crc_high_first(data, length);

}

void create_ack(uint8_t length, uint8_t *data, uint8_t *ack_packet){
    ack_packet[0] = HEADER_VALUE;
    ack_packet[1] = length;
    ack_packet[2] = PACKET_ACK;     //packet id
    ack_packet[3] = data[0];       //ACK or not
    ack_packet[4] = data[1];       //MODE
    ack_packet[5] = crc_high_first((uint8_t*)data, length);
}

uint8_t parse_packet(struct packet *rx, uint8_t c){
    uint8_t correct = 1;
    switch (rx->status) {
        case INIT:
            if (c == HEADER_VALUE){
                rx->index = 0;
                rx->header = c;
                #ifdef DEBUG_PROTOCOL
                printf("HEADER = %d | ", rx->header);
                #endif
                rx->status++;
                break;
            }
            break;
        case GOT_LEN:
            rx->length = c;
            #ifdef DEBUG_PROTOCOL
            printf("%d | ", rx->length);
            #endif
            rx->status++;
            break;
        case GOT_ID:
            rx->packet_id = c;
            #ifdef DEBUG_PROTOCOL
            printf("%d | ", rx->packet_id);
            #endif
            rx->status++;
            break;
        case GOT_DATA:
            rx->data[rx->index] = c;
            #ifdef DEBUG_PROTOCOL
            printf("%d | ", rx->data[rx->index]);
            #endif
            if(rx->index >= (rx->length-1)){
                rx->status++;
            }
            rx->index++;
            break;
        case GOT_CRC:
            rx->crc = c;
            #ifdef DEBUG_PROTOCOL
            printf("%d\n", rx->crc);
            #endif
            rx->status++;
            //break;
        case GOT_PACKET:
            if (crc_high_first((uint8_t*)rx->data, rx->length) != rx->crc){
                rx->crc_fails++;
                correct = 0;
                flags |= FLAG_2;
                printf("CRC FAIL\n");

                printf("HEADER:%d | ", rx->header);
                printf("LENGTH:%d | ", rx->length);
                printf("PACKET ID:%d | ", rx->packet_id);
                printf("MODE:%d | ", rx->data[0]);
                printf("CRC:%d | ", rx->crc);
                printf("CRC_OWN:%d | ", (crc_high_first((uint8_t*)rx->data, rx->length)));
                printf("\n");
            }
            flags |= FLAG_1;
            rx->status = INIT;
            break;

        default:
            rx->status = INIT;
            printf("DEFAULT\n");
            break;
    }
    return correct;
}
