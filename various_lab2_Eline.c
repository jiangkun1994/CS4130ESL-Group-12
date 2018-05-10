 uint8_t mode; //?
 uint16_t BAT_THRESHOLD = 580; //8 or 16?
 uint16_t BAT_WARNING= 600;

 uint32_t timer_panic_mode;
 uint16_t DELAY_MODE_PANIC = 2000; //uint 16?

 enum modes
 {
 	MODE_SAFE = 0,
 	MODE_PANIC,
 	MODE_MANUAL 
 };

 void update_ae(int16_t ae0, int16_t ae1, int16_t ae2, int16_t ae3)
{
	ae[0] = ae0;
	ae[1] = ae1;
	ae[2] = ae2;
	ae[3] = ae3;
}

void set_mode(enum modes m)
{
	switch (m)
	{
		case MODE_SAFE:
			mode = MODE_SAFE;
			printf("Safe mode\n");
			// Turn off motors
			update_ae(0, 0, 0, 0);
			break;

		case MODE_PANIC:
			if (mode != MODE_SAFE)		//?
			{
				printf("Panic mode\n");
				mode = MODE_PANIC;
				// Set motor RPM
				update_ae(175, 175, 175, 175);
				nrf_delay_ms(DELAY_MODE_PANIC);   // Problem with values in buffer
				set_mode(MODE_SAFE);
				break;
			}
			printf("Cannot enter panic mode from safe mode\n");		//? 
			break;

		case MODE_MANUAL:
			printf("Manual mode\n");
			mode = MODE_MANUAL;
			break;

		default: 
			printf("Not a valid mode\n");
			break;
	}
}

uint8_t changing_to_new_mode(uint8_t c)
{
	uint8_t m = c - 48;

	if (m == mode)
	{
		printf("You are already in this mode\n");
		return 0;
	}
	return 1;
}

void init_pc()
{
	set_mode(MODE_SAFE);
}

void check_battery(void)
{
	printf("Battery level: %4d\n", bat_volt);
	printf("Threshold: %d\n", BAT_THRESHOLD);
	if(bat_volt < BAT_THRESHOLD)
	{
		set_mode(MODE_PANIC);
	}
	else if(bat_volt < BAT_WARNING)
	{
		printf("Battery warning\n");
	}
}

void process_key(uint8_t c)
{
	printf("%d\n", c);								// Already in same mode as changing to?
	if(!changing_to_new_mode(c))
	{
		return;
	}
	if(mode == MODE_SAFE)							// And no thrust on joystick? Only abort/exit from safe mode?
	{
		switch (c)
		{
			case '0':
				printf("Already in safe mode\n");
				break;
			case '1':
				printf("Changing to panic mode\n");
				set_mode(MODE_PANIC);
				break;
			case '2':
				printf("Changing to manual mode\n");
				set_mode(MODE_MANUAL);
				break;
			case 27:
				demo_done = true;
				break;
			default:
				nrf_gpio_pin_toggle(RED);
		}
	}
	else if(mode != MODE_SAFE && mode != MODE_PANIC)		
	{
		switch (c)
		{
			case 'q':
				ae[0] += 10;
				break;
			case 'a':
				ae[0] -= 10;
				if (ae[0] < 0) ae[0] = 0;
				break;
			case 'w':
				ae[1] += 10;
				break;
			case 's':
				ae[1] -= 10;
				if (ae[1] < 0) ae[1] = 0;
				break;
			case 'e':
				ae[2] += 10;
				break;
			case 'd':
				ae[2] -= 10;
				if (ae[2] < 0) ae[2] = 0;
				break;
			case 'r':
				ae[3] += 10;
				break;
			case 'f':
				ae[3] -= 10;
				if (ae[3] < 0) ae[3] = 0;
				break;
			
			// Test
			case 't':
				printf("Test\n");
					check_battery();
				break;

			case '0':
				printf("Changing to safe mode\n");
				set_mode(MODE_SAFE);
				break;
			case '1':
				printf("Changing to panic mode\n");
				set_mode(MODE_PANIC);
				break;
			case '2':
				printf("Changing to manual mode\n");
				set_mode(MODE_MANUAL);
				break;

			case 65:
				printf("UP\n");
				break;
			case 66: 
				printf("DOWN\n");
				break;
			case 67:
				printf("RIGHT\n");
				break;
			case 68: 
				printf("LEFT\n");
				break;

			default:
				nrf_gpio_pin_toggle(RED);
		}
	}
	else
	{
		printf("Not possible to change mode\n");
	}
	
}
