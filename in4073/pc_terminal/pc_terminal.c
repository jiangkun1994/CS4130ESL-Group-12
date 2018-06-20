/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <pthread.h>
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
bool read_joystick = true;
uint8_t logging = 0;
unsigned int timer_latency_start = 0, timer_latency_stop = 0;

uint8_t packet_from_pc[MAX_PAYLOAD];
//unsigned int old_time, current_time;
//pthread_mutex_t lock;

#define BAT_THRESHOLD   1050
#define BAT_WARNING		1100

void print_transmission_data(struct msg_telemetry_template *msg_teleRX);
void write_log_to_file(char *filename, struct msg_telemetry_template *msg_teleRX);

/* print the input data from kb and js */
// void print_input_kb_js(){
// 	printf("Control from joystick and keyboard: mode=%d, js_lift=%d, kb_lift_offset=%d, js_yaw=%d, kb_yaw_offset=%d, js_pitch=%d, kb_pitch_offset=%d, js_roll=%d, kb_roll_offset=%d",
// 		mode, js_lift, kb_lift_offset, js_yaw, kb_yaw_offset, js_pitch, kb_pitch_offset, js_roll, kb_roll_offset);
// }

/* sent the packet we created from pc to drone */
void tx_packet(uint8_t *packet){
	int i = 0;
    while(i < packet[1]+PACKET_OVERHEAD){
       	rs232_putchar(packet[i]);
       	i++;
 }
}

// Argument types?
void write_log_to_file(char *filename, struct msg_telemetry_template *msg_teleRX)
{
	//printf("Writing to file\n");
	FILE *f = fopen(filename, "a");
	if (f == NULL)
	{
    printf("Error opening file!\n");
    exit(1);
	}
		fprintf(f, "%d, ", msg_teleRX->Time_stamp);
		fprintf(f, "%d, %d, %d, %d, %d, ", msg_teleRX->mode, msg_teleRX->lift, msg_teleRX->roll, msg_teleRX->pitch, msg_teleRX->yaw);
		fprintf(f, "%d, %d, %d, %d, ", msg_teleRX->engine[0],msg_teleRX->engine[1],msg_teleRX->engine[2],msg_teleRX->engine[3]);
		fprintf(f, "%d, %d, %d, ",msg_teleRX->phi, msg_teleRX->theta, msg_teleRX->psi);
		fprintf(f, "%d, %d, %d, ",msg_teleRX->sp, msg_teleRX->sq, msg_teleRX->sr);
		fprintf(f, "%d, %d, %d, %d, %d, ",msg_teleRX->sax, msg_teleRX->say, msg_teleRX->saz, msg_teleRX->pressure, msg_teleRX->temperature);
		fprintf(f, "%d, %d, %d, %d, %d\n",msg_teleRX->bat_volt, msg_teleRX->P, msg_teleRX->P1, msg_teleRX->P2, msg_teleRX->P3);

		fclose(f);
}

void print_transmission_data(struct msg_telemetry_template *msg_teleRX)
{
	printf("\r%d %4d %4d %4d %4d| ", msg_teleRX->mode, msg_teleRX->lift, msg_teleRX->roll, msg_teleRX->pitch, msg_teleRX->yaw);
	printf("%4d %4d %4d %4d| ", msg_teleRX->engine[0],msg_teleRX->engine[1],msg_teleRX->engine[2],msg_teleRX->engine[3]);
	printf("%6d %6d %6d| ",msg_teleRX->phi, msg_teleRX->theta, msg_teleRX->psi);
	printf("%6d %6d %6d| ",msg_teleRX->sp, msg_teleRX->sq, msg_teleRX->sr);
	printf("%6d %6d %6d %d %d| ",msg_teleRX->sax, msg_teleRX->say, msg_teleRX->saz, msg_teleRX->pressure, msg_teleRX->temperature);
	printf("%4d %2d %2d %2d %2d\n ",msg_teleRX->bat_volt, msg_teleRX->P, msg_teleRX->P1, msg_teleRX->P2, msg_teleRX->P3);
}

void *send_func (){
	int8_t c;
	int8_t next_c;
	struct msg_pc_template msg_pcTX = {0};;
	usleep(2000);
	while(1){
		// while ((c = term_getchar_nb()) != -1){
			// kb_input(c);
			//print_input_kb_js();
		// }
		while ((c = term_getchar_nb()) != -1){
			//printf("c: %c\n", c);
			next_c = term_getchar_nb();
			if(next_c == '['){
				//printf("Arrow\n");
				kb_input(term_getchar_nb());
			}
			else {
				//printf("Not arrow\n");
				kb_input(c);
			}
		}

		if (read_joystick)
			read_js(fd);

		if((connection_failure_flag == 0)){
			pthread_mutex_lock(&lock);
			msg_pcTX.mode 	= mode;
			pthread_mutex_unlock(&lock);
			if(read_joystick)
			{
				if(js_lift == 0)
				{
					msg_pcTX.lift = 0;
					kb_lift_offset = 0;
					kb_roll_offset = 0;
					kb_pitch_offset = 0;
					kb_yaw_offset = 0;
				}
				else
					msg_pcTX.lift 	= inspect_overflow_lift(kb_lift_offset, js_lift);
			}
			else
				msg_pcTX.lift = inspect_overflow_lift(kb_lift_offset, js_lift);
			
			msg_pcTX.roll 	= inspect_overflow(kb_roll_offset, js_roll);
			msg_pcTX.pitch 	= inspect_overflow(kb_pitch_offset, js_pitch);
			msg_pcTX.yaw 		= inspect_overflow(kb_yaw_offset, js_yaw);
			msg_pcTX.P 			= p_adjust;
			msg_pcTX.LOGGING = logging;
			p_adjust = 0;
			create_packet(sizeof(struct msg_pc_template), PACKET_GENERAL, (uint8_t *) &msg_pcTX, packet_from_pc);
			tx_packet(packet_from_pc);
		}
		usleep(20000); // Transmission rate 20 ms
	}
}

void *receive_func (){
	int8_t c;
	int8_t mode_local;
	struct packet rx;
	rx.status = INIT;
	struct msg_telemetry_template *msg_teleRX;
	struct msg_telemetry_template *msg_logRX;
	usleep(2000);
	while(1){

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
						}
						else
						{
							/* Seperate local variable for reading the mode, only 1 pthread_mutex_lock
							   is now needed instead for each read*/
							pthread_mutex_lock(&lock);
							mode_local = mode;
							pthread_mutex_unlock(&lock);

							if (rx.data[1] == CALIBRATION_MODE){
								pthread_mutex_lock(&lock);
								mode = SAFE_MODE;
								pthread_mutex_unlock(&lock);
							}

							else if (rx.data[1] == 26){
								timer_latency_stop = mon_time_ms();
								unsigned int diff = timer_latency_stop - timer_latency_start;
								printf("LATENCY = %d\n", diff);
							}
							/* Local mode used here */
							else if(mode_local != SAFE_MODE && rx.data[1] == END_MODE){
								mode = PANIC_MODE;
							}
							else{
								pthread_mutex_lock(&lock);
								mode = rx.data[1];
								pthread_mutex_unlock(&lock);
							}

							//printf("MODE CHANGED CORRECTLY TO %d\n", rx.data[1]);
						}
						break;
					case PACKET_TELEMETRY:
						msg_teleRX = (struct msg_telemetry_template *)&rx.data[0];

						if(msg_teleRX->bat_volt < BAT_THRESHOLD)
						{
							printf("Battery voltage below threshold, entering panic mode\n");
						}
						else if(msg_teleRX->bat_volt < BAT_WARNING)
						{
							printf("Warning. Low battery.\n");
						}

						print_transmission_data(msg_teleRX);

						// printf("\r%d %4d %4d %4d %4d| ", msg_teleRX->mode, msg_teleRX->lift, msg_teleRX->roll, msg_teleRX->pitch, msg_teleRX->yaw);
						// printf("%4d %4d %4d %4d| ", msg_teleRX->engine[0],msg_teleRX->engine[1],msg_teleRX->engine[2],msg_teleRX->engine[3]);
						// printf("%6d %6d %6d| ",msg_teleRX->phi, msg_teleRX->theta, msg_teleRX->psi);
						// printf("%6d %6d %6d| ",msg_teleRX->sp, msg_teleRX->sq, msg_teleRX->sr);
						// printf("%6d %6d %6d %d %d| ",msg_teleRX->sax, msg_teleRX->say, msg_teleRX->saz, msg_teleRX->pressure, msg_teleRX->temperature);
						// printf("%4d %2d %2d %2d\n ",msg_teleRX->bat_volt, msg_teleRX->P, msg_teleRX->P1, msg_teleRX->P2);

						break;
					case PACKET_LOG:
						//printf("Log received\n" );
						msg_logRX = (struct msg_telemetry_template *)&rx.data[0];
						//print_transmission_data(msg_logRX);
						write_log_to_file("log-data.csv", msg_logRX);
						break;
					default:
						printf("UNDEFINED PACKET RECIEVED\n");
						break;
				}
				flags &= ~FLAG_1;
			}
	}
}
/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	pthread_t receive_thread, send_thread;

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");
	js_init();
	term_initio();
	rs232_open();
	protocol_init();
	term_puts("Type ^C to exit\n");

		/* discard any incoming text
	 */
	// while ((c = rs232_getchar_nb()) != -1)
	// 	fputc(c,stderr);

	pthread_create(&receive_thread, NULL, *receive_func, NULL);
	pthread_create(&send_thread, NULL, *send_func, NULL);

	while(1){
	}

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	if(pthread_join(receive_thread, NULL)){
		fprintf(stderr, "Error joining receive thread\n");
		return 2;
	}

	if(pthread_join(send_thread, NULL)){
		fprintf(stderr, "Error joining send thread\n");
		return 2;
	}

	pthread_mutex_destroy(&lock);

	return 0;
}
