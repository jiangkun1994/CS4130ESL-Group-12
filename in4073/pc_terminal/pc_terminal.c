/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <ctype.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>
#include "../protocol/protocol.h"
#include "console_rs232.h"
#include "joystick.h"
#include "keyboard.h"
#include "variable.h"

//Global mode and fd definition
char mode = 0;
int fd = 0;
uint8_t p_adjust = 0;

/* print the input data from kb and js */
void print_input_kb_js(){
	printf("Control from joystick and keyboard: mode=%d, js_lift=%d, kb_lift_offset=%d, js_yaw=%d, kb_yaw_offset=%d, js_pitch=%d, kb_pitch_offset=%d, js_roll=%d, kb_roll_offset=%d",
		mode, js_lift, kb_lift_offset, js_yaw, kb_yaw_offset, js_pitch, kb_pitch_offset, js_roll, kb_roll_offset);
}

/* sent the packet we created from pc to drone */
void tx_packet(int8_t *packet){
	int i = 0;
    while(i < packet[1]+PACKET_OVERHEAD){
       	rs232_putchar(packet[i]);
       	i++;
 }
}
/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	int8_t c;
	int8_t data_general[LENGTH_GENERAL];
	int8_t packet_from_pc[MAX_PAYLOAD+PACKET_OVERHEAD];
	struct packet rx;

	//js_init();

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	protocol_init();

	term_puts("Type ^C to exit\n");

	unsigned int old_time, current_time;
	old_time = mon_time_ms();

		/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	while(1){

        //read_js(fd);

        while ((c = term_getchar_nb()) != -1){
        	kb_input(c);
        	//print_input_kb_js();
        }

        current_time = mon_time_ms();
        if((current_time - old_time) > 10){
        	data_general[0] = mode;
        	data_general[1] = inspect_overflow_1ift(kb_lift_offset, 0);
        	data_general[2] = inspect_overflow(kb_pitch_offset, 0);
        	data_general[3] = inspect_overflow(kb_roll_offset, 0);
        	data_general[4] = inspect_overflow(kb_yaw_offset, 0);

          //printf("PACKET PC|%d|%d|%d|%d|%d|\n", data_general[0], data_general[1], data_general[2], data_general[3], data_general[4]);

        	create_packet(LENGTH_GENERAL, PACKET_GENERAL, p_adjust, data_general, packet_from_pc);
       		tx_packet(packet_from_pc);

			old_time = current_time;

        }

        if ((c = rs232_getchar_nb()) != -1){
					if (rx.status == INIT && c != -HEADER_VALUE) {
						//printf("DATA:%d\n", c);
						term_putchar(c);
					}
        	parse_packet (&rx, c);

	        if (flags & FLAG_1){
						switch(rx.packet_id){
							case PACKET_ACK:
								if (rx.data[0] & FLAG_2){
									printf("NEED TO re-transmit\n");
									tx_packet(packet_from_pc);
									old_time = mon_time_ms();
								}
								break;
							case PACKET_TELEMETRY:
								mode = rx.data[0];
								printf("DRONE TELEMETRY: ");
								int i = 0;
								while (i < LENGTH_TELEMETRY){
											printf("|%d", rx.data[i]);
											i++;
								}
								//mode = rx.data[0];

								printf("\n");
								break;
						default:
								printf("UNDEFINED PACKET RECIEVED\n");
								break;
						}
						flags &= ~FLAG_1;
					}
        }
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}
