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
uint8_t connection_failure_flag = 0;

/* print the input data from kb and js */
void print_input_kb_js(){
	printf("Control from joystick and keyboard: mode=%d, js_lift=%d, kb_lift_offset=%d, js_yaw=%d, kb_yaw_offset=%d, js_pitch=%d, kb_pitch_offset=%d, js_roll=%d, kb_roll_offset=%d",
		mode, js_lift, kb_lift_offset, js_yaw, kb_yaw_offset, js_pitch, kb_pitch_offset, js_roll, kb_roll_offset);
}

/* sent the packet we created from pc to drone */
void tx_packet(uint8_t *packet){
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
	uint8_t packet_from_pc[MAX_PAYLOAD];
	struct packet rx;
	struct msg_pc_template msg_pcTX = {0};;
	struct msg_telemetry_template *msg_teleRX;
	rx.status = INIT;

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
        if(((current_time - old_time) > 10) && (connection_failure_flag == 0)){
					msg_pcTX.mode 	= mode;
					msg_pcTX.lift 	= inspect_overflow_1ift(kb_lift_offset, 0);
					msg_pcTX.roll 	= inspect_overflow(kb_roll_offset, 0);
					msg_pcTX.pitch 	= inspect_overflow(kb_pitch_offset, 0);
					msg_pcTX.yaw 		= inspect_overflow(kb_yaw_offset, 0);
					msg_pcTX.P 			= p_adjust;
					p_adjust = 0;

					create_packet(sizeof(struct msg_pc_template), PACKET_GENERAL, (uint8_t *) &msg_pcTX, packet_from_pc);
       				tx_packet(packet_from_pc);

					old_time = current_time;

        }

				//if ((c = rs232_getchar_nb()) != -1){
				c = rs232_getchar_nb();
					if (rx.status == INIT && c != HEADER_VALUE && c!= -1) {
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
								else
								{
									if (rx.data[1] == CALIBRATION_MODE)
										mode = SAFE_MODE;
									else
										mode = rx.data[1];
									//printf("MODE CHANGED CORRECTLY TO %d\n", rx.data[1]);
								}
								break;
							case PACKET_TELEMETRY:
								msg_teleRX = (struct msg_telemetry_template *)&rx.data[0];

								printf("\r%d %4d %4d %4d %4d| ", msg_teleRX->mode, msg_teleRX->lift, msg_teleRX->roll, msg_teleRX->pitch, msg_teleRX->yaw);
								printf("%4d %4d %4d %4d| ", msg_teleRX->engine[0],msg_teleRX->engine[1],msg_teleRX->engine[2],msg_teleRX->engine[3]);
								printf("%6d %6d %6d| ",msg_teleRX->phi, msg_teleRX->theta, msg_teleRX->psi);
								printf("%6d %6d %6d| ",msg_teleRX->sp, msg_teleRX->sq, msg_teleRX->sr);
								printf("%6d %6d %6d| ",msg_teleRX->sax, msg_teleRX->say, msg_teleRX->saz);
								printf("%4d %2d %2d %2d\n ",msg_teleRX->bat_volt, msg_teleRX->P, msg_teleRX->P1, msg_teleRX->P2);
								break;
						default:
								printf("UNDEFINED PACKET RECIEVED\n");
								break;
						}
						flags &= ~FLAG_1;
					}
        //}
}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}
