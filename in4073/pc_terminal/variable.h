#ifndef VARIABLE_H__
#define VARIABLE_H__

//#include "../protocol/protocol.h"

char kb_yaw_offset;
char kb_pitch_offset;
char kb_roll_offset;
char kb_lift_offset;

// char kb_yaw;
// char kb_pitch;
// char kb_roll;
// char kb_lift;

char js_yaw;
char js_pitch;
char js_roll;
char js_lift;

// char yaw_offset_p_up;
// char yaw_offset_p_down;
// char roll_pitch_offset_p1;
// char roll_pitch_offset_p2;

extern char mode;
extern uint8_t p_adjust;
extern bool logging;

int	axis[6];
int	button[12];
extern int fd;
extern uint8_t connection_failure_flag;


//struct packet mypacket;

#endif
