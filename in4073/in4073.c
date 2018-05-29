/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "protocol/protocol.h"
#include "drone.h"

#define BAT_THRESHOLD   500
#define BAT_WARNING			501

#define MIN_RPM 184320 // 180 RPM
#define MAX_RPM 460800 // 450 RPM

uint8_t telemetry_packet[MAX_PAYLOAD];
struct msg_telemetry_template msg_teleTX = {0};
static struct packet rx = {0};
struct msg_pc_template *msg_pcRX;
uint8_t panic_loops = 0;

uint32_t time_latest_packet_us, cur_time_us;

//in this function calculate the values for the ae[] array makis
void calculate_rpm(int Z, int L, int M, int N)
{
	// Z = lift
	// L = row
	// M = pitch
	// N = yaw
	int ae1[4],i;
	//if there is lift force calculate ae[] array values
	if(Z>0)
	{
		// //calculate the square of each motor rpm
		// ae1[0] = 0.25*(Z + 2*M - N) + MIN_RPM;
		// ae1[1] = 0.25*(Z - 2*L + N) + MIN_RPM;
		// ae1[2] = 0.25*(Z - 2*M - N) + MIN_RPM;
		// ae1[3] = 0.25*(Z + 2*L + N) + MIN_RPM;


		// ae1[0] = 0.25*(Z + 2*M - N); // test without min RPM
		// ae1[1] = 0.25*(Z - 2*L + N);
		// ae1[2] = 0.25*(Z - 2*M - N);
		// ae1[3] = 0.25*(Z + 2*L + N);

		ae1[0] = Z + M - N;
		ae1[1] = Z - L + N;
		ae1[2] = Z - M - N;
		ae1[3] = Z + L + N;
		//printf("z:%d \t l:%d \t m:%d \t n:%d\n", Z, L, M, N);

		//minimum rpm
		for(i=0;i<4;i++)
		{
			if(ae1[i]<MIN_RPM)
			{
				ae1[i]=MIN_RPM;
			}
		}
		//maximum rpm
		for(i=0;i<4;i++)
		{
			if(ae1[i]>MAX_RPM)
			{
				ae1[i]=MAX_RPM;
			}
		}

		// scale the values into those can be used in reality
		ae[0]=ae1[0] >> 10;
		ae[1]=ae1[1] >> 10;
		ae[2]=ae1[2] >> 10;
		ae[3]=ae1[3] >> 10;
	}
	//if there is no lift force everything should be shut down
	else if(Z<=0)
	{
		ae[0]=0;
		ae[1]=0;
		ae[2]=0;
		ae[3]=0;
	}
	//update motors
	run_filters_and_control();
}

/*------------------------------------------------------------------
 * send_ack -- send acknowledgement
 *------------------------------------------------------------------
 */
void send_ack(uint8_t data)
{
	uint8_t ack_packet[PACKET_ACK_LENGTH];
	//create_ack(LENGTH_ACK, pc_packet.p_adjust, data, ack_packet);
	create_ack(LENGTH_ACK, data, ack_packet);

	int i = 0;
	while (i < PACKET_ACK_LENGTH){
		uart_put(ack_packet[i]);
		i++;
	}
}

/*----------------------------------------------------------------------
 * send_telemetry -- send the data read from sensors from drone to pc
 *----------------------------------------------------------------------
 */
void send_telemetry()
{
	if (check_telemetry_timer_flag())
	{
		msg_teleTX.mode = cur_mode;
		msg_teleTX.lift = cur_lift;
		msg_teleTX.roll = cur_roll;
		msg_teleTX.pitch = cur_pitch;
		msg_teleTX.yaw = cur_yaw;

		msg_teleTX.engine[0] = ae[0];
		msg_teleTX.engine[1] = ae[1];
		msg_teleTX.engine[2] = ae[2];
		msg_teleTX.engine[3] = ae[3];

		msg_teleTX.phi = phi;
		msg_teleTX.theta = theta;
		msg_teleTX.psi = psi;

		msg_teleTX.sp = sp;
		msg_teleTX.sq = sq;
		msg_teleTX.sr = sr;

		msg_teleTX.sax = sax;
		msg_teleTX.say = say;
		msg_teleTX.saz = saz;

		msg_teleTX.bat_volt = bat_volt;

		msg_teleTX.P = p;

		create_packet(sizeof(struct msg_telemetry_template), PACKET_TELEMETRY, (uint8_t *) &msg_teleTX, telemetry_packet);

		int i = 0;
		while(i < telemetry_packet[1]+PACKET_OVERHEAD){
			uart_put(telemetry_packet[i]);
			i++;
		}
		clear_telemetry_timer_flag();
	}
}

/* process the packet from pc */
void handle_transmission_data()
{
	if (rx_queue.count){
		parse_packet (&rx, dequeue(&rx_queue));
	}

	if (flags & FLAG_1){
		//printf("receive succesful via flag on drone\n");
		if(!(flags & FLAG_2)){
			switch(rx.packet_id) {
				case PACKET_GENERAL:
				msg_pcRX = (struct msg_pc_template *)&rx.data[0];

				pc_packet.data[0] = msg_pcRX->mode;
				pc_packet.data[1] = msg_pcRX->lift;
				pc_packet.data[2] = msg_pcRX->pitch;
				pc_packet.data[3] = msg_pcRX->roll;
				pc_packet.data[4] = msg_pcRX->yaw;
				pc_packet.p_adjust = msg_pcRX->P;
				default:
					break;
			}
			time_latest_packet_us = get_time_us();
		}
		//send_ack(flags);
		flags &= ~FLAG_2;

		flags &= ~FLAG_1;
	}
}

/* if there is no packet over 600ms, drone will get into the panic mode */
void check_connection()
{
	if (check_connection_timer_flag())
	{
		uint32_t time_diff;
		cur_time_us = get_time_us();
		time_diff = cur_time_us - time_latest_packet_us;
		if((time_diff > 600000) && cur_mode != SAFE_MODE)
		{
			connection = false;
			statefunc = panic_mode;
		}
		clear_connection_timer_flag();
	}
}

void manual_mode()
{
	cur_mode = MANUAL_MODE;

	//indicate that you are in manual mode
	nrf_gpio_pin_write(RED,1);
	nrf_gpio_pin_write(YELLOW,0);

	//check_connection();
	//check_battery();

	//read the new messages to come
	handle_transmission_data();
	//printf("PK_manual|%d|%d|%d|%d|%d|\n", pc_packet.data[0], pc_packet.data[1], pc_packet.data[2], pc_packet.data[3], pc_packet.data[4]);
	switch (pc_packet.data[0])
	{
		case PANIC_MODE:
			statefunc = panic_mode;
			break;
		case MANUAL_MODE:
			cur_lift = pc_packet.data[1];
			cur_pitch = pc_packet.data[2];
			cur_roll = pc_packet.data[3];
			cur_yaw = pc_packet.data[4];
			break;
		default:
			//printf("Not a valid mode\n");
			break;
	}

	//if there is a new command do the calculations
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 13; // test them on drone to find the suitable parameters
		// if(cur_roll > 63)
		// {
		// 	roll_moment = ((cur_roll - 128) * 2000);
		// }
		//else{
		roll_moment = cur_roll << 11;
		//}
		// if(cur_pitch > 63)
		// {
		// 	pitch_moment = ((cur_pitch - 128) * 2000);
		// }
		//else{
		pitch_moment = cur_pitch << 11;
		//}
		// if(cur_yaw > 63)
		// {
		// 	yaw_moment = ((cur_yaw - 128) * 4000);
		// }
		//else{
		yaw_moment = cur_yaw << 12;
		//}

		calculate_rpm(lift_force,roll_moment,pitch_moment,yaw_moment);

		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
		//print your changed state
		//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],bat_volt);
	}

	//while there is no message received wait here and check your connection
	// while(msg==false && connection==true)
	// {
	// 	check_connection();
	// }

	//read the new messages to come
	// handle_transmission_data();
	// //printf("PK_manual|%d|%d|%d|%d|%d|\n", pc_packet.data[0], pc_packet.data[1], pc_packet.data[2], pc_packet.data[3], pc_packet.data[4]);
	// switch (pc_packet.data[0])
	// {
	// 	case PANIC_MODE:
	// 		statefunc=panic_mode;
	// 		break;
	// 	case MANUAL_MODE:
	// 		cur_lift=pc_packet.data[1];
	// 		cur_pitch=pc_packet.data[2];
	// 		cur_roll=pc_packet.data[3];
	// 		cur_yaw=pc_packet.data[4];
	// 		break;
	// 	default:
	// 		//printf("Not a valid mode\n");
	// 		break;
	// }
}

/* For the drone in zero-movement, there is a non-zero offset, so need to measure this offset which can be used to get more accurate sensor value */
void calibration_mode() // PROBLEM: repeat the calibrarion mode, the mode does not syncronize???  Maybe the function sent_telemetry should in each mode or it is the problem of check telemetry time lag.
{
	cur_mode = CALIBRATION_MODE;

	sp_off = 0;
	sq_off = 0;
	sr_off = 0;
	phi_off = 0;
	theta_off = 0;

	//indicate that you are in calibration mode
	nrf_gpio_pin_write(RED,1);
	nrf_gpio_pin_write(YELLOW,1);
	nrf_gpio_pin_write(GREEN,0);

	// the number of samples read from sensors
	int sample = 0;

	// get the sum of offset for 150 samples
	while(sample < 150)
	{
		if(check_sensor_int_flag()) // ???
		{
			get_dmp_data();
			//clear_sensor_int_flag();
			sp_off = sp_off + sp;
			sq_off = sq_off + sq;
			sr_off = sr_off + sr;
			phi_off = phi_off + phi;
			theta_off = theta_off + theta;
			sample++;
		}
	}

	// calculate the average off set for 150 samples
	sp_off = sp_off / 150;
	sq_off = sq_off / 150;
	sr_off = sr_off / 150;
	phi_off = phi_off / 150;
	theta_off = theta_off / 150;

	// when in calibration mode, no parse packet, so add this to indicate you are still in conncection
	time_latest_packet_us = get_time_us();

	printf("sp_off: %d, sq_off: %d, sr_off: %d, phi_off: %d, theta_off: %d \n", sp_off,sq_off,sr_off,phi_off,theta_off);
	printf("CALIBRATION MODE FINISHED! \n");

	// handle_transmission_data();

	// switch(pc_packet.data[0])
	// {
	// 	case SAFE_MODE:
	// 		safe_print = true;
	// 		statefunc = safe_mode;
	// 		break;
	// 	case PANIC_MODE:
	// 		statefunc = panic_mode;
	// 		break;
	// 	default:
	// 		break;
	// }
	safe_print = true;
	statefunc = safe_mode;
}

void yaw_control_mode()
{
	cur_mode = YAW_CONTROL_MODE;

	//indicate that you are in yaw control mode
	nrf_gpio_pin_write(RED,0);
	nrf_gpio_pin_write(YELLOW,1);
	nrf_gpio_pin_write(GREEN,0);

	//check_connection();
	if(check_sensor_int_flag())
	{
		get_dmp_data();
		calculate_rpm(lift_force, roll_moment, pitch_moment, p * (yaw_moment - sr)); // Not sure that whether the sr should multiply a constant or not
	}

	if (check_timer_flag())
	{
		if (counter++%15 == 0)
		{
			nrf_gpio_pin_toggle(BLUE);

			//battery_check();

			//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, p=%d, bat_volt=%d p_adjust=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],p,bat_volt, pc_packet.p_adjust);
		}
		clear_timer_flag();
	}

	handle_transmission_data();

	switch (pc_packet.data[0])
	{
		case PANIC_MODE:
			lift_force = 0;
			roll_moment = 0;
			pitch_moment = 0;
			yaw_moment = 0;
			p = 40;
			statefunc = panic_mode;
			break;
		case YAW_CONTROL_MODE:
			cur_lift = pc_packet.data[1];
			cur_pitch = pc_packet.data[2];
			cur_roll = pc_packet.data[3];
			cur_yaw = pc_packet.data[4];
			if(pc_packet.p_adjust == P_YAW_UP)
			{
				p += 1;
				if(p >= 127)
				{
					p = 127;
				}
				pc_packet.p_adjust = 0;
			}
			else if(pc_packet.p_adjust == P_YAW_DOWN)
			{
				p -= 1;
				if(p <= 0)
				{
					p = 0;
				}
				pc_packet.p_adjust = 0;
			}
			break;
		default:
			//printf("Not a valid mode\n");
			break;
	}

	//if there is a new command do the calculations
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 13; // test them on drone to find the suitable parameters
		// if(cur_roll > 63)
		// {
		// 	roll_moment = ((cur_roll - 128) * 2000);
		// }
		//else{
		roll_moment = cur_roll << 11;
		//}
		// if(cur_pitch > 63)
		// {
		// 	pitch_moment = ((cur_pitch - 128) * 2000);
		// }
		//else{
		pitch_moment = cur_pitch << 11;
		//}
		// if(cur_yaw > 63)
		// {
		// 	yaw_moment = ((cur_yaw - 128) * 4000);
		// }
		//else{
		yaw_moment = cur_yaw << 12;
		//}

		//calculate_rpm(lift_force,roll_moment,pitch_moment,yaw_moment);

		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
		//print your changed state
		//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, p=%d, bat_volt=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],p,bat_volt);
	}

}

void full_control_mode()
{
	cur_mode = FULL_CONTROL_MODE;

	//indicate that you are in full control mode
	nrf_gpio_pin_write(RED,1);
	nrf_gpio_pin_write(YELLOW,0); // yellow and gree lights are on
	nrf_gpio_pin_write(GREEN,0);

	if(check_sensor_int_flag())
	{
		get_dmp_data();
		//clear_sensor_int_flag();
		calculate_rpm(lift_force, p1 * (roll_moment - (phi - phi_off)) - p2 * (sp - sp_off), p1 * (pitch_moment - (theta - theta_off)) - p2 * (sq - sq_off), p * (yaw_moment - (sr - sr_off)));
	}

	if (check_timer_flag())
	{
		if (counter++%15 == 0)
		{
			nrf_gpio_pin_toggle(BLUE);

			//battery_check();

			//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, p=%d, p1=%d, p2=%d, bat_volt=%d \n", cur_mode, ae[0],ae[1],ae[2],ae[3],p,p1,p2,bat_volt);
		}
		clear_timer_flag();
	}

	handle_transmission_data();
	switch(pc_packet.data[0])
	{
		case PANIC_MODE:
			lift_force = 0;
			roll_moment = 0;
			pitch_moment = 0;
			yaw_moment = 0;
			p = 40;
			p1 = 10;
			p2 = 10;
			statefunc = panic_mode;
			break;
		case FULL_CONTROL_MODE:
			cur_lift = pc_packet.data[1];
			cur_pitch = pc_packet.data[2];
			cur_roll = pc_packet.data[3];
			cur_yaw = pc_packet.data[4];
			if(pc_packet.p_adjust == P_YAW_UP)
			{
				p += 1;
				if(p >= 127)
				{
					p = 127;
				}
				pc_packet.p_adjust = 0;
			}
			if(pc_packet.p_adjust == P_YAW_DOWN)
			{
				p -= 1;
				if(p <= 0)
				{
					p = 0;
				}
				pc_packet.p_adjust = 0;
			}
			if(pc_packet.p_adjust == P1_ROLL_PITCH_UP)
			{
				p1 += 1;
				if(p1 >= 127)
				{
					p1 = 127;
				}
				pc_packet.p_adjust = 0;
			}
			if(pc_packet.p_adjust == P1_ROLL_PITCH_DOWN)
			{
				p1 -= 1;
				if(p1 <= 0)
				{
					p1 = 0;
				}
				pc_packet.p_adjust = 0;
			}
			if(pc_packet.p_adjust == P2_ROLL_PITCH_UP)
			{
				p2 += 1;
				if(p2 >= 127)
				{
					p2 = 127;
				}
				pc_packet.p_adjust = 0;
			}
			if(pc_packet.p_adjust == P2_ROLL_PITCH_DOWN)
			{
				p2 -= 1;
				if(p2 <= 0)
				{
					p2 = 0;
				}
				pc_packet.p_adjust = 0;
			}
			break;
		default:
			printf("Not a valid mode!!\n");
			break;
	}

	//if there is a new command do the calculations
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 13; // test them on drone to find the suitable parameters
		// if(cur_roll > 63)
		// {
		// 	roll_moment = ((cur_roll - 128) * 2000);
		// }
		//else{
		roll_moment = cur_roll << 11;
		//}
		// if(cur_pitch > 63)
		// {
		// 	pitch_moment = ((cur_pitch - 128) * 2000);
		// }
		//else{
		pitch_moment = cur_pitch << 11;
		//}
		// if(cur_yaw > 63)
		// {
		// 	yaw_moment = ((cur_yaw - 128) * 4000);
		// }
		//else{
		yaw_moment = cur_yaw << 12;
		//}

		//calculate_rpm(lift_force,roll_moment,pitch_moment,yaw_moment);

		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
		//print your changed state
		//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, p=%d, p1=%d, p2=%d, bat_volt=%d \n", cur_mode, ae[0],ae[1],ae[2],ae[3],p,p1,p2,bat_volt);
	}


}

//panic mode state makis
void panic_mode()
{
	// if(cur_mode == SAFE_MODE){
	// 	return;
	// }
	cur_mode = PANIC_MODE;

	//indicate that you are in panic mode
	nrf_gpio_pin_write(RED,0);
	nrf_gpio_pin_write(YELLOW,0);

	if (check_panic_mode_timer_flag())
	{
		panic_loops++;
		//Update latest time since where not parsing any incomming data
		time_latest_packet_us = get_time_us();
		//fly at minimum rpm
		if(ae[0] > 175 || ae[1] > 175 || ae[2] > 175 || ae[3] > 175)
		{
			ae[0] -= 10;
			ae[1] -= 10;
			ae[2] -= 10;
			ae[3] -= 10;
			run_filters_and_control();
			//nrf_delay_ms(200);
			//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],bat_volt);
		}
		else if (panic_loops > MIN_PANIC_LOOPS)
		{
			//zero down some values
			cur_lift = 0;
			cur_pitch = 0;
			cur_roll = 0;
			cur_yaw = 0;
			old_lift = 0;
			old_pitch = 0;
			old_roll = 0;
			old_yaw = 0;

			//print your changed state
			//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],bat_volt);

			//after 2 seconds get to safe mode
			//nrf_delay_ms(2000);

			//fixes a bug, doesn't care to check connection going to safe mode anyway
			//time_latest_packet_us = get_time_us();

			//flag to print once in safe mode
			safe_print = true;
			//enters safe mode
			statefunc = safe_mode;
			panic_loops = 0;
		}
		clear_panic_mode_timer_flag();
	}
}

//safe mode state makis
void safe_mode()
{
	cur_mode = SAFE_MODE;

	// indicate that your connection is broken
	if(connection == false)
	{
		nrf_gpio_pin_write(RED,0);
		nrf_gpio_pin_write(YELLOW,0);
		nrf_gpio_pin_write(GREEN,0);
	}
	else
	{
		// indicate that you are in safe mode
		nrf_gpio_pin_write(RED,0);
		nrf_gpio_pin_write(YELLOW,1);
		nrf_gpio_pin_write(GREEN,1);
	}

	//motors are shut down
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;
	run_filters_and_control();
	// if(safe_print==true)
	// {
	// 	printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],bat_volt);
	// 	safe_print=false;
	// }

	//check_connection();
	//check_battery();
	if(safe_print == true)
	{
		//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",cur_mode,ae[0],ae[1],ae[2],ae[3],bat_volt);
		safe_print = false;
	}


	//if there is battery and the connection is ok read the messages
	if(battery == true && connection == true)
	{
		handle_transmission_data();
		//printf("PK_safe|%d|%d|%d|%d|%d|\n", pc_packet.data[0], pc_packet.data[1], pc_packet.data[2], pc_packet.data[3], pc_packet.data[4]);
		switch (pc_packet.data[0])
		{
			//check for not switching to manual mode with offsets different than zero
			case MANUAL_MODE:
				if(pc_packet.data[1] == 0 && pc_packet.data[2] == 0 && pc_packet.data[3] == 0 && pc_packet.data[4] == 0)
				{
					//print your changed state
					//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",MANUAL_MODE,ae[0],ae[1],ae[2],ae[3],bat_volt);
					statefunc = manual_mode;
				}
				break;
			case SAFE_MODE:
				break;
			case CALIBRATION_MODE:
				statefunc = calibration_mode;
				break;
			case YAW_CONTROL_MODE:
				if(pc_packet.data[1] == 0 && pc_packet.data[2] == 0 && pc_packet.data[3] == 0 && pc_packet.data[4] == 0)
				{
					//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, bat_volt=%d \n",YAW_CONTROL_MODE,ae[0],ae[1],ae[2],ae[3],bat_volt);
					statefunc = yaw_control_mode;
				}
				break;
			case FULL_CONTROL_MODE:
				if(pc_packet.data[1] == 0 && pc_packet.data[2] == 0 && pc_packet.data[3] == 0 && pc_packet.data[4] == 0)
				{
					//printf("DRONE SIDE: mode=%d, ae[0]=%d, ae[1]=%d, ae[2]=%d, ae[3]=%d, p=%d, p1=%d, p2=%d, bat_volt=%d \n", cur_mode, ae[0],ae[1],ae[2],ae[3],p,p1,p2,bat_volt);
					statefunc = full_control_mode;
				}
				break;
			default:
				//printf("Not a valid mode - DATA[0]=%d\n", pc_packet.data[0]);
				break;
		}
	}
}

void initialize()
{
	//message flag initialization
	//msg=false;

	//drone modules initialization
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);
	baro_init();
	spi_flash_init();
	adc_request_sample();
	//ble_init();
	demo_done = false;
	//battery = true;
	//adc_request_sample();

	//initialise the pc_packet struct to safe values, just in case
	pc_packet.data[0] = SAFE_MODE;
	pc_packet.data[1] = 0; // lift
	pc_packet.data[2] = 0; // pitch
	pc_packet.data[3] = 0; // roll
	pc_packet.data[4] = 0; // yaw
	rx.status = INIT;

	//initialise rest of the variables to safe values, just in case
	cur_mode = SAFE_MODE;
	cur_lift = 0;
	cur_pitch = 0;
	cur_roll = 0;
	cur_yaw = 0;
	old_lift = 0;
	old_pitch = 0;
	old_roll = 0;
	old_yaw = 0;
	lift_force = 0;
	roll_moment = 0;
	pitch_moment = 0;
	yaw_moment = 0;
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;
	battery = true;
	connection = true;
	safe_print = true;
	p = 40;
	p1 = 10;
	p2 = 10;
	// p_ctrl=10;
	//first get to safe mode
	statefunc = safe_mode;
	counter = 0;
	//set_mode(SAFE_MODE);
}

void check_battery()
{
	//check battery voltage
	if (check_timer_flag())
	{
		//clear_timer_flag();
		adc_request_sample();

		if ((bat_volt < BAT_THRESHOLD) && battery)
		{
			printf("bat voltage %d below threshold %d\n",bat_volt,BAT_THRESHOLD);
			battery = false;
			statefunc = panic_mode;
			//set_mode(PANIC_MODE);
		}
		else if(bat_volt < BAT_WARNING)
		{
			printf("Battery warning, battery %d\n", bat_volt);
		}
		else if ((bat_volt > BAT_THRESHOLD) && !battery)
		{
			battery = true;
			printf("Battery true %d\n", bat_volt);
		}
		clear_timer_flag();
	}
}

/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */
int main(void){

	initialize();

	while (!demo_done){

		check_battery();

		//get to the state
		(*statefunc)();

		send_telemetry();
		check_connection();
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
