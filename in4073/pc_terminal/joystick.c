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

/* Kun Jiang */
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

/* Kun Jiang */
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

/* Kun Jiang */
void js_give_packet(char action, char value){
	switch(action){
		case LIFT:
		  js_lift = value;
		break;
		case PITCH:
		  js_pitch = value;
		break;
		case ROLL:
		  js_roll = value;
		break;
		case YAW:
		  js_yaw = value;
		break;
	}
}

/* Kun Jiang */
void set_js_packet(char action, int axis){
	char give_value;
	give_value = js_scale_values(axis);
	js_give_packet(action, give_value);

}

/* Kun Jiang */
void set_js_packet_lift(char action, unsigned int axis){
	char give_value;
	give_value = js_scale_values_lift(axis);
	js_give_packet(action, give_value);

}


void js_init(){
	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
      read_joystick = false;
      js_lift = 0;
      js_roll = 0;
      js_pitch = 0;
      js_yaw = 0;
      printf("NO JOYSTICK FOUND, RUNING PROGRAM IN KEYBOARD MODE ONLY\n");
	}

	/* non-blocking mode
	 */
	fcntl(fd, F_SETFL, O_NONBLOCK);
}

/* Modified by Kun Jiang */
/* read the data from joystick (button or handler) */
int read_js(int fd)
{
	struct js_event js;
	unsigned int total;

	/* check up on JS
	 */
	while (read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event)){

		/* register data
		 */
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

	/* the data from handler */
	// roll
	if(axis[0] < -JS_MIN_VALUE || axis[0] > JS_MIN_VALUE){
		set_js_packet(ROLL, axis[0]);
	}
	else{
		set_js_packet(ROLL, 0);
	}

	// pitch
	if(axis[1] < -JS_MIN_VALUE || axis[1] > JS_MIN_VALUE){
		set_js_packet(PITCH, axis[1]);
	}
	else{
		set_js_packet(PITCH, 0);
	}

	// yaw
	if(axis[2] < -JS_MIN_VALUE || axis[2] > JS_MIN_VALUE){
		set_js_packet(YAW, axis[2]);
	}
	else{
		set_js_packet(YAW, 0);
	}

	// lift
	total = 65534 - (axis[3] + 32767);
    if (total > JS_MIN_VALUE)
    {
        set_js_packet_lift(LIFT, total);
    }
    else
    {
        set_js_packet_lift(LIFT, 0);
    }
    return 0;
}
