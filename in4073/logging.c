/* All functions: Eline Stenwig */
#include "in4073.h"

#define MAX_FLASH_ADDRESS 0x01FFFF

uint32_t address = 0;
uint32_t address_read = 0;

void write_mission_data()
{
	if((address + sizeof(struct msg_telemetry_template)) > MAX_FLASH_ADDRESS)
	{
		flash_chip_erase();
		address = 0;
		address_read = 0;
	}

	if(check_log_timer_flag())
	{
		update_telemetry_data();
		if(flash_write_bytes(address, (uint8_t *)&msg_teleTX, sizeof(struct msg_telemetry_template)) == true)
		{
			address += sizeof(struct msg_telemetry_template);
		}
		clear_log_timer_flag();
	}
}

void read_mission_data()
{
	uint8_t j = 0;
	uint8_t log_packet[MAX_PAYLOAD];
	uint8_t data[sizeof(struct msg_telemetry_template)];

	while(flash_read_bytes(address_read, data, sizeof(struct msg_telemetry_template)) == true)
	{
		create_packet(sizeof(struct msg_telemetry_template), PACKET_LOG, data, log_packet);
		while(j < log_packet[1] + PACKET_OVERHEAD)
		{
			uart_put(log_packet[j]);
			j++;
		}
		j = 0;
		address_read += sizeof(struct msg_telemetry_template);
		if(address_read >= address)
		{
			printf("\n\nREADING IS DONE\n\n");
			break;
		}
		nrf_delay_ms(5);
	}
}
