#ifndef KEYBOARD_H__
#define KEYBOARD_H__

#include <stdio.h>
#include <inttypes.h>
#include "variable.h"
#include "../protocol/protocol.h"

/* mode on keyboard */
#define ZERO      0x30
#define ONE		    0x31
#define TWO		    0x32
#define THREE	    0x33
#define FOUR	    0x34
#define FIVE	    0x35
#define SIX       0x36
#define SEVEN     0x37

/* action on keyboard */
#define A 		    0x61
#define Z 		    0x7A
#define VK_LEFT 	0x44 //NOT SURE ABOUT THE ARROW BUTTON
#define VK_UP		  0x41
#define VK_RIGHT	0x43
#define VK_DOWN		0x42
#define Q		      0x71
#define W		      0x77

/* abort/exit */
#define ESC		    0x1B


/* functions in keyboard.c */
void kb_input(uint8_t input_key);
char inspect_overflow(char offset, char js);
char inspect_overflow_lift(char offset, char js);

#endif
