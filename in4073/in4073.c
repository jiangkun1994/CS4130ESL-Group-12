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

#define BAT_THRESHOLD   1050

uint8_t telemetry_packet[MAX_PAYLOAD];
struct msg_telemetry_template msg_teleTX = {0};
static struct packet rx = {0};
struct msg_pc_template *msg_pcRX;
uint8_t panic_loops = 0;
uint32_t time_latest_packet_us, cur_time_us;
uint8_t calibration_flag;

//
void send_ack(uint8_t *data);
void send_mode_ack(uint8_t mode);
void update_telemetry_data(void);
void send_telemetry(void);
void handle_transmission_data(void);
void check_connection(void);

// Update control
void update_actions_manual_control(void);
void update_actions_yaw_control(void);
void update_actions_full_control(void);
void update_actions_height_control(void);

// Modes
void end_mode(void);
void height_control_mode_end(void);
void manual_mode(void);								// Green, blue, red
void calibration_mode(void);					// Red, yellow, green
void yaw_control_mode(void);					// Red, blue
void full_control_mode(void);					// Red, yellow
void height_control_mode(void);				// Yellow
void panic_mode(void);								// Green, blue, yellow
void safe_mode(void);									// No connection: No lights, connection: Blue, red, yellow

void run_modes(void);

void initialize(void);
void check_battery(void);

/* Randy Prozée */
/*------------------------------------------------------------------
 * send_ack -- send acknowledgement
 *------------------------------------------------------------------
 */
void send_ack(uint8_t *data)
{
	uint8_t ack_packet[PACKET_ACK_LENGTH];
	create_ack(LENGTH_ACK, data, ack_packet);

	int i = 0;
	while (i < PACKET_ACK_LENGTH){
		uart_put(ack_packet[i]);
		i++;
	}
}

/* Eline Stenwig */
void send_mode_ack(uint8_t mode)
{
	uint8_t data[2];
	data[0] = FLAG_1;
	data[1] = mode;
	send_ack(data);
}

/* Randy Prozée */
void update_telemetry_data(void)
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

	msg_teleTX.phi = phi; 				// roll angle
	msg_teleTX.theta = theta; 		// pitch angle
	msg_teleTX.psi = psi; 				// yaw angle

	msg_teleTX.sp = sp; 					// roll angular rate
	msg_teleTX.sq = sq; 					// pitch angular rate
	msg_teleTX.sr = sr; 					// yaw angular rate

	msg_teleTX.sax = sax; 				// x direction velocity
	msg_teleTX.say = say; 				// y direction velocity
	msg_teleTX.saz = saz; 				// z direction velocity

	msg_teleTX.bat_volt = bat_volt;

	msg_teleTX.P = p;
	msg_teleTX.P1 = p1;
	msg_teleTX.P2 = p2;
	msg_teleTX.P3 = p3;
	msg_teleTX.pressure = pressure;
	msg_teleTX.temperature = temperature;

	msg_teleTX.Time_stamp = get_time_us();
}

/* Randy Prozée */
/*----------------------------------------------------------------------
 * send_telemetry -- send the data read from sensors from drone to pc
 *----------------------------------------------------------------------
 */
void send_telemetry(void)
{
	if (check_telemetry_timer_flag())
	{
		update_telemetry_data();

		create_packet(sizeof(struct msg_telemetry_template), PACKET_TELEMETRY, (uint8_t *) &msg_teleTX, telemetry_packet);

		int i = 0;
		while(i < telemetry_packet[1]+PACKET_OVERHEAD){
			uart_put(telemetry_packet[i]);
			i++;
		}
		clear_telemetry_timer_flag();
	}
}

/* Randy Prozée */
/* process the packet from pc */
void handle_transmission_data(void)
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

					if (msg_pcRX-> mode != pc_packet.data[0])
					{
						pc_packet.data[0] = msg_pcRX->mode;
						uint8_t data[2];
						data[0] = flags;
						data[1] = pc_packet.data[0];
						send_ack(data); //Only if mode change, send ack
					}

					pc_packet.data[1] = msg_pcRX->lift;
					pc_packet.data[2] = msg_pcRX->pitch;
					pc_packet.data[3] = msg_pcRX->roll;
					pc_packet.data[4] = msg_pcRX->yaw;
					pc_packet.p_adjust = msg_pcRX->P;
					pc_packet.logging = msg_pcRX->LOGGING;
					break;
				default:
					break;
			}
			time_latest_packet_us = get_time_us();
		}
		flags &= ~FLAG_2;

		flags &= ~FLAG_1;
	}
}

/* Randy Prozée */
// If there is no packet over 600ms, drone will get into the panic mode
void check_connection(void)
{
	if (check_connection_timer_flag())
	{
		uint32_t time_diff;
		cur_time_us = get_time_us();
		time_diff = cur_time_us - time_latest_packet_us;
		if((time_diff > 600000) && cur_mode != SAFE_MODE && cur_mode != END_MODE)
		{
			connection = false;
			statefunc = PANIC_MODE;
			printf("CONNECTION IS LOST, ENTERING PANIC MODE\n");
		}
		clear_connection_timer_flag();
	}
}

/* Kun Jiang */
void update_actions_manual_control(void)
{
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 14; // test them on drone to find the suitable parameters
		roll_moment = cur_roll << 11;
		pitch_moment = cur_pitch << 11;
		yaw_moment = cur_yaw << 12;
		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
	}
}

/* Kun Jiang */
void update_actions_yaw_control(void)
{
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 14; // test them on drone to find the suitable parameters
		roll_moment = cur_roll << 11;
		pitch_moment = cur_pitch << 11;
		yaw_moment = cur_yaw << 9;
		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
	}
}

/* Kun Jiang */
void update_actions_full_control(void)
{
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 14; // test them on drone to find the suitable parameters
		roll_moment = cur_roll << 9;
		pitch_moment = cur_pitch << 9;
		yaw_moment = cur_yaw << 9;
		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
	}
}

/* Kun Jiang */
void update_actions_height_control(void)
{
	if(old_lift != cur_lift || old_pitch != cur_pitch || old_roll != cur_roll || old_yaw != cur_yaw)
	{
		lift_force = cur_lift << 13; // test them on drone to find the suitable parameters
		roll_moment = cur_roll << 12;
		pitch_moment = cur_pitch << 12;
		yaw_moment = cur_yaw << 13;
		old_lift = cur_lift;
		old_roll = cur_roll;
		old_pitch = cur_pitch;
		old_yaw = cur_yaw;
	}
}

/* Eline Stenwig */
void end_mode(void)
{
	cur_mode = END_MODE;
	read_mission_data();
	demo_done = true;
}

/* Eline Stenwig */
void height_control_mode_end(void)
{
	cur_mode = HEIGHT_CONTROL_MODE_END;
	send_mode_ack(prev_mode);
	statefunc = prev_mode;
}

/* Kun Jiang */
void manual_mode(void)
{
	cur_mode = MANUAL_MODE;

	//indicate that you are in manual mode
	nrf_gpio_pin_write(GREEN,1);
	nrf_gpio_pin_write(BLUE,1);
	nrf_gpio_pin_write(RED,1);
	nrf_gpio_pin_write(YELLOW,0);

	//read the new messages to come
	handle_transmission_data();

	switch (pc_packet.data[0])
	{
		case PANIC_MODE:
			statefunc = PANIC_MODE;
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
	update_actions_manual_control();
	calculate_rpm(lift_force, roll_moment, pitch_moment, yaw_moment);
}

/* Eline Stenwig */
/* For the drone in zero-movement, there is a non-zero offset, so need to measure this offset which can be used to get more accurate sensor value */
void calibration_mode(void) // what is the advice from TA about calibration mode? Some initial values from DMP are not right?
{
	static uint16_t sample = 0;
	cur_mode = CALIBRATION_MODE;

	if(sample == 0)
	{
		sp_off_ = 0;
		sq_off_ = 0;
		sr_off_ = 0;
		phi_off_ = 0;
		theta_off_ = 0;

		//indicate that you are in calibration mode
		nrf_gpio_pin_write(RED,1);
		nrf_gpio_pin_write(YELLOW,1);
		nrf_gpio_pin_write(GREEN,1);
		nrf_gpio_pin_write(BLUE,0);
	}

	handle_transmission_data();

	if(check_sensor_int_flag())
	{
		int16_t pre_phi = phi;
		int16_t pre_theta = theta;
		get_dmp_data();

		int16_t diff_phi = pre_phi - phi;
		int16_t diff_theta = pre_theta - theta;

		// Need to check if data from DMP are junk data
		if (CHECK_RANGE(sp, sq, sr, 5) && CHECK_RANGE(0, diff_phi, diff_theta, 2)){
			sp_off_ = sp_off_ + sp;
			sq_off_ = sq_off_ + sq;
			sr_off_ = sr_off_ + sr;
			phi_off_ = phi_off_ + phi;
			theta_off_ = theta_off_ + theta;
			sample++;
		}
	}

	if (sample >= 600){
		// calculate the average off set for 600 samples
		sp_off = (int16_t)(sp_off_ / 600);
		sq_off = (int16_t)(sq_off_ / 600);
		sr_off = (int16_t)(sr_off_ / 600);
		phi_off = (int16_t)(phi_off_ / 600);
		theta_off = (int16_t)(theta_off_ / 600);
		printf("sp_off: %d, sq_off: %d, sr_off: %d, phi_off: %d, theta_off: %d \n", sp_off,sq_off,sr_off,phi_off,theta_off);
		printf("CALIBRATION MODE FINISHED! \n");
		statefunc = SAFE_MODE;
		calibration_flag = true;
		sample = 0;
	}
}

/* Kun Jiang */
void yaw_control_mode(void)
{
	cur_mode = YAW_CONTROL_MODE;

	//indicate that you are in yaw control mode
	nrf_gpio_pin_write(RED,1);
	nrf_gpio_pin_write(BLUE,1);
	nrf_gpio_pin_write(YELLOW,0);
	nrf_gpio_pin_write(GREEN,0);

	if(check_sensor_int_flag())
	{
		get_dmp_data();
		calculate_rpm(lift_force, roll_moment, pitch_moment, p * (yaw_moment + (sr << 4))); // Not sure that whether the sr should multiply a constant or not
	}

	handle_transmission_data();

	switch (pc_packet.data[0])
	{
		case PANIC_MODE:
			p = 1;
			statefunc = PANIC_MODE;
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
	update_actions_yaw_control();
}

/* Kun Jiang */
void full_control_mode(void)
{
	cur_mode = FULL_CONTROL_MODE;

	//indicate that you are in full control mode
	nrf_gpio_pin_write(RED,1);
	nrf_gpio_pin_write(YELLOW,1);
	nrf_gpio_pin_write(BLUE,0);
	nrf_gpio_pin_write(GREEN,0);

	if(check_sensor_int_flag())
	{
		get_dmp_data();
		calculate_rpm(lift_force,
			p1 * (roll_moment - (phi - phi_off)) - (p2 << 2)* (sp - sp_off),
			p1 * (pitch_moment - (theta - theta_off)) + (p2 << 2) * (sq - sq_off),
			p * (yaw_moment + ((sr - sr_off) << 4)));
	}   // cascaded p (coupled): p2 * (p1 * (roll_moment - (phi - phi_off)) - (sp - sp_off))

	handle_transmission_data();

	switch(pc_packet.data[0])
	{
		case PANIC_MODE:
			p = 1;
			p1 = 1;
			p2 = 1;
			statefunc = PANIC_MODE;
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
		case HEIGHT_CONTROL_MODE:
			// Change p's?
			prev_mode = FULL_CONTROL_MODE;
			statefunc = HEIGHT_CONTROL_MODE;
			break;
		default:
			//printf("Not a valid mode!!\n");
			break;
	}

	//if there is a new command do the calculations
	update_actions_full_control();
}

/* Eline Stenwig - switching to/from the mode, not control*/
void height_control_mode(void)
{
	cur_mode = HEIGHT_CONTROL_MODE;

	//indicate that you are in height control mode
	nrf_gpio_pin_write(RED,0);
	nrf_gpio_pin_write(YELLOW,1);
	nrf_gpio_pin_write(GREEN,0);
	nrf_gpio_pin_write(BLUE,0);

	//check_connection();
	if(check_sensor_int_flag())
	{
		read_baro();
		// This part is not done
		calculate_rpm(p3 * (lift_force - pressure), roll_moment, pitch_moment, yaw_moment); // Not sure that whether the sr should multiply a constant or not
	}

	handle_transmission_data();

	switch (pc_packet.data[0])
	{
		case PANIC_MODE:
			p3 = 1;
			statefunc = PANIC_MODE;
			break;
		case HEIGHT_CONTROL_MODE:
			if(cur_lift != pc_packet.data[1]){
				cur_lift = pc_packet.data[1];			//?
				statefunc = HEIGHT_CONTROL_MODE_END;
				break;
			}
			cur_pitch = pc_packet.data[2];
			cur_roll = pc_packet.data[3];
			cur_yaw = pc_packet.data[4];

			if(pc_packet.p_adjust == P3_HEIGHT_UP)
			{
				p3 += 1;
				if(p3 >= 127)
				{
					p3 = 127;
				}
				pc_packet.p_adjust = 0;
			}
			else if(pc_packet.p_adjust == P3_HEIGHT_DOWN)
			{
				p3 -= 1;
				if(p3 <= 0)
				{
					p3 = 0;
				}
				pc_packet.p_adjust = 0;
			}
			break;
		case HEIGHT_CONTROL_MODE_END:
			statefunc = HEIGHT_CONTROL_MODE_END;
			break;
		case FULL_CONTROL_MODE:
			statefunc = FULL_CONTROL_MODE;
			break;
		default:
			//printf("Not a valid mode\n");
			break;
	}

	//if there is a new command do the calculations
	update_actions_height_control();
}

/* Kun Jiang */
//panic mode state makis
void panic_mode(void)
{
	cur_mode = PANIC_MODE;

	//indicate that you are in panic mode
	nrf_gpio_pin_write(RED,0);
	nrf_gpio_pin_write(YELLOW,1);
	nrf_gpio_pin_write(BLUE,1);
	nrf_gpio_pin_write(GREEN,1);

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
			for(int i=0;i<4;i++) //  to prevent the negative value for RPM
			{
				if(ae[i] <= 0)
					ae[i] = 0;
			}
			run_filters_and_control();
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
			lift_force = 0;
			roll_moment = 0;
			pitch_moment = 0;
			yaw_moment = 0;

			//fixes a bug, doesn't care to check connection going to safe mode anyway
			time_latest_packet_us = get_time_us();

			//flag to print once in safe mode
			//enters safe mode
			statefunc = SAFE_MODE;
			panic_loops = 0;
		}
		clear_panic_mode_timer_flag();
	}
}

/* Kun Jiang */
//safe mode state makis
void safe_mode(void)
{
	cur_mode = SAFE_MODE;

	// indicate that your connection is broken
	if(connection == false)
	{
		nrf_gpio_pin_write(BLUE,0);
		nrf_gpio_pin_write(RED,0);
		nrf_gpio_pin_write(YELLOW,0);
		nrf_gpio_pin_write(GREEN,0);
	}
	else
	{
		// indicate that you are in safe mode
		nrf_gpio_pin_write(BLUE,1);
		nrf_gpio_pin_write(RED,1);
		nrf_gpio_pin_write(YELLOW,1);
		nrf_gpio_pin_write(GREEN,0);
	}

	//motors are shut down
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;
	run_filters_and_control();

	//if there is battery and the connection is ok read the messages
	if(battery == true && connection == true)
	{
		handle_transmission_data();
		if(pc_packet.p_adjust == LOGGING_DATA)
		{
			read_mission_data();
			pc_packet.p_adjust = 0;
		}

		switch (pc_packet.data[0])
		{
			case END_MODE:
				statefunc = END_MODE;
				break;
			//check for not switching to manual mode with offsets different than zero
			case MANUAL_MODE:
				if(pc_packet.data[1] == 0 && pc_packet.data[2] == 0 && pc_packet.data[3] == 0 && pc_packet.data[4] == 0)
				{
					statefunc = MANUAL_MODE;
				}
				break;
			case SAFE_MODE:
				break;
			case CALIBRATION_MODE:
				statefunc = CALIBRATION_MODE;
				break;
			case YAW_CONTROL_MODE:
				if(pc_packet.data[1] == 0 && pc_packet.data[2] == 0 && pc_packet.data[3] == 0 && pc_packet.data[4] == 0)
				{
					statefunc = YAW_CONTROL_MODE;
				}
				break;
			case FULL_CONTROL_MODE:
				if(pc_packet.data[1] == 0 && pc_packet.data[2] == 0 && pc_packet.data[3] == 0 && pc_packet.data[4] == 0)
				{
					statefunc = FULL_CONTROL_MODE;
				}
				break;
			default:
				//printf("Not a valid mode - DATA[0]=%d\n", pc_packet.data[0]);
				break;
		}
	}
}

void initialize(void)
{

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

	demo_done = false;

	//initialise the pc_packet struct to safe values, just in case
	pc_packet.data[0] = SAFE_MODE;
	pc_packet.data[1] = 0; // lift
	pc_packet.data[2] = 0; // pitch
	pc_packet.data[3] = 0; // roll
	pc_packet.data[4] = 0; // yaw
	pc_packet.logging = 0;
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

	p = 1;
	p1 = 1;
	p2 = 1;
	p3 = 1;

	calibration_flag = false;

	//first get to safe mode
	statefunc = SAFE_MODE;;
}

/* Eline Stenwig */
void check_battery(void)
{
	//check battery voltage
	if (check_timer_flag())
	{
		adc_request_sample();

		if ((bat_volt < BAT_THRESHOLD) && battery)
		{
			battery = false;
			if(statefunc != SAFE_MODE){
				statefunc = PANIC_MODE;
				send_mode_ack(PANIC_MODE);
			}
		}
		else if ((bat_volt > BAT_THRESHOLD) && !battery)
		{
			battery = true;
		}
		clear_timer_flag();
	}
}

/* Kun Jiang */
void run_modes(void)
{
	switch (statefunc) {
		case END_MODE:
			end_mode();
			break;
		case SAFE_MODE:
			safe_mode();
			break;
		case MANUAL_MODE:
			manual_mode();
			break;
		case PANIC_MODE:
			panic_mode();
			break;
		case YAW_CONTROL_MODE:
			yaw_control_mode();
			break;
		case CALIBRATION_MODE:
			calibration_mode();
			break;
		case FULL_CONTROL_MODE:
			full_control_mode();
			break;
		case HEIGHT_CONTROL_MODE:
			height_control_mode();
			break;
		case HEIGHT_CONTROL_MODE_END:
			height_control_mode_end();
			break;
		default:
			panic_mode();
			break;
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

		if(pc_packet.logging == 1)
		{
			write_mission_data();
		}

		run_modes();

		send_telemetry();
		check_connection();
	}

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
