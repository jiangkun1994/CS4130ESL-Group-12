#include "joystick.h"


/* current axis and button readings
 */

unsigned int mon_time_ms(void)
{
        unsigned int    ms;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
        ms = ms + tv.tv_usec / 1000;
        return ms;
}

void mon_delay_ms(unsigned int ms)
{
        struct timespec req, rem;

        req.tv_sec = ms / 1000;
        req.tv_nsec = 1000000 * (ms % 1000);
        assert(nanosleep(&req,&rem) == 0);
}


char js_scale_values(int axis){
	char scale_axis;
	int PreRange = 65535;
	int NewRange = 127; // 0011 1111
	int PreMax = 32767;
	int PreMin = -32768;
	int NewMin = -64;

	if(axis >= PreMin && axis <= PreMax){
		if(axis == 0){
			scale_axis = 0;
		}
		else{
			scale_axis = (((axis - PreMin) * NewRange) / PreRange) + NewMin;
		}
	}
	return scale_axis;
}

char js_scale_values_lift(int axis){
	char scale_axis;
	int PreRange = 65535;
	int NewRange = 127;
	int PreMax = 65535;
	int PreMin = 0;
	int NewMin = 0;

	if(axis >= PreMin && axis <= PreMax){
		if(axis == 0){
			scale_axis = 0;
		}
		else{
			scale_axis = (((axis - PreMin) * NewRange) / PreRange) + NewMin;
		}
	}
	return scale_axis;
}


void js_give_packet(char action, char value){
	switch(action){
		case LIFT:
		js_lift = value;
		//printf("LIFT: %d\n", js_lift);
		break;
		case PITCH:
		js_pitch = value;
		//printf("PITCH: %d\n", js_pitch);
		break;
		case ROLL:
		js_roll = value;
		//printf("ROLL: %d\n", js_roll);
		break;
		case YAW:
		js_yaw = value;
		//printf("YAW: %d\n", js_yaw);
		break;
	}
}

// void set_js_packet(char action, int axis, int divisor){
// 	float throttle = 0;
// 	float amplifier = 127;
// 	char give_value;
//
// 	throttle = (axis + (divisor / 2)) / divisor;
//
// 	if(throttle > 1000)
// 		throttle = 1000;
// 	else if(throttle < -1000)
// 		throttle = -1000;
//
// 	give_value = throttle / 1000.0 * amplifier;
// 	js_give_packet(action, give_value);
//
// }

void set_js_packet(char action, int axis){
	char give_value;
	give_value = js_scale_values(axis);
	js_give_packet(action, give_value);

}

void set_js_packet_lift(char action, unsigned int axis){
	char give_value;
	give_value = js_scale_values_lift(axis);
	js_give_packet(action, give_value);

}


//#define JS_DEV	"/dev/input/js0"

void js_init(){
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		//perror("jstest");
    //exit(1);
    // printf("NO JOYSTICK FOUND, DO YOU WANT TO RUN IN KEYBOARD MODE ONLY [y/n]: ");
    // char r;
    // r = getchar();
    // while(r != 'n' && r != 'N' && r != 'y' && r != 'Y')
    // {
    //   printf("invalid input, enter the choice(y/Y/n/N) again : ");
    //   r = getchar();
    //   if (r == '\n') r = getchar();
    // }
    //
    // if (r == 'n' || r == 'N'){
    //   printf("EXITING PROGRAM\n");
    //   exit(1);
    // }
    // else if (r == 'y'|| r == 'Y')
    // {
      read_joystick = false;
      js_lift = 0;
      js_roll = 0;
      js_pitch = 0;
      js_yaw = 0;
      printf("NO JOYSTICK FOUND, RUNING PROGRAM IN KEYBOARD MODE ONLY\n");
    //}
	}

	/* non-blocking mode
	 */
	fcntl(fd, F_SETFL, O_NONBLOCK);
}

/* read the data from joystick (button or handler) */
int read_js(int fd)
{
	//int 		fd;
	struct js_event js;
	unsigned int total;
	//int cc;

	/* check up on JS
	 */
	while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){

		/* register data
		 */
		// fprintf(stderr,".");
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
		}
	}
	if (errno != EAGAIN) {
		perror("\njs: error reading (EAGAIN)");
		exit (1);
	}

	/* choose the mode after pressing the button*/
	if (button[0]){
    pthread_mutex_lock(&lock);
		mode = PANIC_MODE;
    pthread_mutex_unlock(&lock);
  }
	// if (button[1])
	// 	mode = SAFE_MODE;
	// if (button[2]) // Do we need to check whether data are clear here?
	// 	mode = MANUAL_MODE;
	// if (button[3])
	// 	mode = CALIBRATION_MODE;
	// if (button[4])
	// 	mode = YAW_CONTROL_MODE;
	// if (button[5])
	// 	mode = FULL_CONTROL_MODE;


	/* the data from handler */
	// roll
	if(axis[0] < -JS_MIN_VALUE || axis[0] > JS_MIN_VALUE){
		set_js_packet(ROLL, axis[0]);
	}
	else{
		set_js_packet(ROLL, 0);
	}

	//set_js_packet(ROLL, axis[0]);

	// pitch
	if(axis[1] < -JS_MIN_VALUE || axis[1] > JS_MIN_VALUE){
		set_js_packet(PITCH, axis[1]);
	}
	else{
		set_js_packet(PITCH, 0);
	}

	//set_js_packet(PITCH, axis[1]);

	// yaw
	if(axis[2] < -JS_MIN_VALUE || axis[2] > JS_MIN_VALUE){
		set_js_packet(YAW, axis[2]);
	}
	else{
		set_js_packet(YAW, 0);
	}

	//set_js_packet(YAW, axis[2]);

	// lift
	total = 65534 - (axis[3] + 32767);
	//cc = total / 2 ;
    if (total > JS_MIN_VALUE)
    {
        set_js_packet_lift(LIFT, total);
    }
    else
    {
        set_js_packet_lift(LIFT, 0);
    }

    //set_js_packet(LIFT, axis[3]);


    return 0;
}
