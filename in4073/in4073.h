/*------------------------------------------------------------------
 *  in4073.h -- defines, globals, function prototypes
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#ifndef IN4073_H__
#define IN4073_H__

#include <inttypes.h>
#include <stdio.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "ml.h"
#include "app_util_platform.h"
#include "protocol/protocol.h"
#include <math.h>
#include "drone.h"

#define RED				22
#define YELLOW		24
#define GREEN			28
#define BLUE			30
#define INT_PIN		5

#define CHECK_RANGE(x,y,z,n) ( (x > -n && x < n && y > -n && y < n && z > -n && z < n) ? 1 : 0)
bool demo_done;

// for data logging
extern uint32_t address;
extern uint32_t address_read;

// Control
int16_t motor[4],ae[4];
void run_filters_and_control();

// Timers
#define TIMER_PERIOD						50 	//50ms=20Hz (MAX 23bit, 4.6h)
#define PANIC_MODE_PERIOD				200	//200ms
#define TELEMETRY_TIMER_PERIOD 	500	//500ms
#define CONNECTION_MODE_PERIOD	400 //400ms
#define LOG_PERIOD							10	//10 ms
#define MIN_PANIC_LOOPS					4

void timers_init(void);
uint32_t get_time_us(void);
bool check_timer_flag(void);
void clear_timer_flag(void);
bool check_telemetry_timer_flag(void);
void clear_telemetry_timer_flag(void);
bool check_panic_mode_timer_flag(void);
void clear_panic_mode_timer_flag(void);
bool check_connection_timer_flag(void);
void clear_connection_timer_flag(void);
bool check_log_timer_flag(void);
void clear_log_timer_flag(void);


// GPIO
void gpio_init(void);

// Queue
#define QUEUE_SIZE 256
typedef struct {
	uint8_t Data[QUEUE_SIZE];
	uint16_t first,last;
  uint16_t count;
} queue;

void init_queue(queue *q);
void enqueue(queue *q, char x);
char dequeue(queue *q);

// UART
#define RX_PIN_NUMBER  16
#define TX_PIN_NUMBER  14
queue rx_queue;
queue tx_queue;
uint32_t last_correct_checksum_time;
void uart_init(void);
void uart_put(uint8_t);

// TWI
#define TWI_SCL	4
#define TWI_SDA	2
void twi_init(void);
bool i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t const *data);
bool i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data);

// MPU wrapper
int16_t phi, theta, psi;
int16_t sp, sq, sr;
int16_t sax, say, saz;
uint8_t sensor_fifo_count;
void imu_init(bool dmp, uint16_t interrupt_frequency); // if dmp is true, the interrupt frequency is 100Hz - otherwise 32Hz-8kHz
void get_dmp_data(void);
void get_raw_sensor_data(void);
bool check_sensor_int_flag(void);
void clear_sensor_int_flag(void);

// Barometer
int32_t pressure;
int32_t temperature;
void read_baro(void);
void baro_init(void);

// ADC
uint16_t bat_volt;
void adc_init(void);
void adc_request_sample(void);

// Flash
bool spi_flash_init(void);
bool flash_chip_erase(void);
bool flash_write_byte(uint32_t address, uint8_t data);
bool flash_write_bytes(uint32_t address, uint8_t *data, uint32_t count);
bool flash_read_byte(uint32_t address, uint8_t *buffer);
bool flash_read_bytes(uint32_t address, uint8_t *buffer, uint32_t count);

// BLE
queue ble_rx_queue;
queue ble_tx_queue;
volatile bool radio_active;
void ble_init(void);
void ble_send(void);

// Testing
void write_mission_data(void);
void read_mission_data(void);
void update_telemetry_data(void);


#endif // IN4073_H__
