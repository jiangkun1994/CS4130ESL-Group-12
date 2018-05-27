//declare functions
// int calculate_Z(char lift);
// int calculate_L(char roll);
// int calculate_M(char pitch);
// int calculate_N(char yaw);
void calculate_rpm(int Z, int L, int M, int N);
//void calibration_mode();
void manual_mode();
void safe_mode();
void panic_mode();
void calibration_mode();
void yaw_control_mode();
void full_control_mode();
//void check_connection();
//void process_input();

//state pointer
void (*statefunc)();
//void (*setmode)();

//variable to hold current mode
char cur_mode;

//p controller value
unsigned char p;
unsigned char p1;
unsigned char p2;

//variable to hold current movement
int8_t cur_lift;
int8_t cur_pitch;
int8_t cur_roll;
int8_t cur_yaw;

//variable to hold old movement
int8_t old_lift;
int8_t old_pitch;
int8_t old_roll;
int8_t old_yaw;

//force and moments in drone
int lift_force;
int roll_moment;
int pitch_moment;
int yaw_moment;

//dc offset of gyro sensor
int16_t sp_off;
int16_t sq_off;
int16_t sr_off;
int16_t phi_off;
int16_t theta_off;

//counters to take care of exiting when communication breaks down
extern uint32_t time_latest_packet_us, cur_time_us;

//flags indicating that there is still connection and battery
bool connection;
bool battery;

//flag indicating that a new message has arrived
//bool msg;

//flag to print in safe mode
bool safe_print;

uint32_t counter;
