#ifndef VARIABLE_H__
#define VARIABLE_H__

#include <pthread.h>

char kb_yaw_offset;
char kb_pitch_offset;
char kb_roll_offset;
char kb_lift_offset;

char js_yaw;
char js_pitch;
char js_roll;
char js_lift;


extern char mode;
extern uint8_t p_adjust;
extern uint8_t logging;

int	axis[6];
int	button[12];
pthread_mutex_t lock;
extern int fd;
extern uint8_t connection_failure_flag;
extern unsigned int timer_latency_start, timer_latency_stop;

#endif
