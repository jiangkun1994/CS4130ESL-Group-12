#include "keyboard.h"

/* check whether the combining data from kb and js is overflow for roll, yaw, pitch
* & 0x7F to make sure the first bit in every data byte is 0,
* preventing the problem of the same byte as header */
char inspect_overflow(char offset, char js)
{
    char temp_sum;
    if ((offset + js) > 63)
    {
        temp_sum = 63;
    }
    else if ((offset + js) < -63)
    {
        temp_sum = -63;
    }
    else
    {
        temp_sum = offset + js;
    }
    return temp_sum;
}

/* check overflow for lift */
char inspect_overflow_1ift(char offset, char js)
{
    char temp_sum;

    if ((offset + js) > 127)
    {
        temp_sum = 127;
    }
    else if ((offset + js) < 0)
    {
        temp_sum = 0;
    }
    else
    {
        temp_sum = offset + js;
    }
    return temp_sum;
}

/* process the input from keyboard */
void kb_input(uint8_t input_key){
	switch(input_key){
		case ZERO:
			mode = SAFE_MODE;
			break;
		case ONE:
			mode = PANIC_MODE;
			break;
		case TWO:
			if (
				inspect_overflow_1ift(kb_lift_offset, js_lift) == 0 &&
				inspect_overflow(kb_pitch_offset, js_pitch) == 0 &&
				inspect_overflow(kb_roll_offset, js_roll) == 0 &&
				inspect_overflow(kb_yaw_offset, js_yaw) == 0)
				mode = MANUAL_MODE;
			else
				printf("The control data from kb and js are not zero!! Please press button c and check js\n");
			break;
        case THREE:
            mode = CALIBRATION_MODE;
            break;
        case FOUR:
            if (
                inspect_overflow_1ift(kb_lift_offset, js_lift) == 0 &&
                inspect_overflow(kb_pitch_offset, js_pitch) == 0 &&
                inspect_overflow(kb_roll_offset, js_roll) == 0 &&
                inspect_overflow(kb_yaw_offset, js_yaw) == 0)
                mode = YAW_CONTROL_MODE;
            else
                printf("The control data from kb and js are not zero!! Please press button c and check js\n");
            break;
        case FIVE:
            if (
                inspect_overflow_1ift(kb_lift_offset, js_lift) == 0 &&
                inspect_overflow(kb_pitch_offset, js_pitch) == 0 &&
                inspect_overflow(kb_roll_offset, js_roll) == 0 &&
                inspect_overflow(kb_yaw_offset, js_yaw) == 0)
                mode = FULL_CONTROL_MODE;
            else
                printf("The control data from kb and js are not zero!! Please press button c and check js\n");
            break;
        case SEVEN:
            if (
                inspect_overflow_1ift(kb_lift_offset, js_lift) == 0 &&
                inspect_overflow(kb_pitch_offset, js_pitch) == 0 &&
                inspect_overflow(kb_roll_offset, js_roll) == 0 &&
                inspect_overflow(kb_yaw_offset, js_yaw) == 0)
                mode = HEIGHT_CONTROL_MODE;
            else
                printf("The control data from kb and js are not zero!! Please press button c and check js\n");
            break;
		case ESC:
			mode = PANIC_MODE;
			break;
		case 'a': // lift (engine RPM, keyboard: a/z, joystick: throttle)
        	if(kb_lift_offset!=127)
        	{
            	kb_lift_offset+=1;
        	}
        	break;
		case 'z':
        	if(kb_lift_offset!=-127)
        	{
            	kb_lift_offset-=1;
        	}
        	break;
		case 'q': // yaw (yaw, keyboard: q/w, joystick: twist handle)
        	if(kb_yaw_offset!=-63)
        	{
            	kb_yaw_offset-=1;
        	}
        	break;
		case 'w':
        	if(kb_yaw_offset!=63)
        	{
            	kb_yaw_offset+=1;
        	}
        	break;
    	case 'c':
      		{
      		    kb_yaw_offset = 0;
                kb_lift_offset = 0;
                kb_roll_offset = 0;
                kb_pitch_offset = 0;
      		}
			break;
    case 'f':
          printf("CONNECTION FAILURE TESTING\n");
          connection_failure_flag = 1;
          break;
		case 'D': // VK_LEFT: // roll (roll, keyboard: left/right arrows, joystick: handle left/right (x))
        	if(kb_roll_offset!=63)
        	{
            	kb_roll_offset+=1;
        	}
        	break;
		case 'A': // VK_UP: // pitch (pitch, keyboard: up/down arrows, joystick: handle forward/backward (y))
        	if(kb_pitch_offset!=-63)
        	{
            	kb_pitch_offset-=1;
        	}
        	break;
		case 'C': // VK_RIGHT:
        	if(kb_roll_offset!=-63)
        	{
            	kb_roll_offset-=1;
        	}
        	break;
		case 'B': // VK_DOWN:
		    if(kb_pitch_offset!=63)
        	{
            	kb_pitch_offset+=1;
        	}
        	break;
        case 'u':
            p_adjust = P_YAW_UP;
            break;
        case 'j':
            p_adjust = P_YAW_DOWN;
            break;
        case 'i':
            p_adjust = P1_ROLL_PITCH_UP;
            break;
        case 'k':
            p_adjust = P1_ROLL_PITCH_DOWN;
            break;
        case 'o':
            p_adjust = P2_ROLL_PITCH_UP;
            break;
        case 'l':
            p_adjust = P2_ROLL_PITCH_DOWN;
            break;
        case '-':
            p_adjust = P3_HEIGHT_DOWN;
            break;
        case '=':
            p_adjust = P3_HEIGHT_UP;
            break;
        case 'm':
            p_adjust = LOGGING_DATA;
            break;
        case 'p':
            if(logging == 1)
                logging = 0;
            else
                logging = 1;
            break;
        default:
        	//return;
        	break;
	}
}
