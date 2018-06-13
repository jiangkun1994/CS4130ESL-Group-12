//declare functions
// int calculate_Z(char lift);
// int calculate_L(char roll);
// int calculate_M(char pitch);
// int calculate_N(char yaw);
void calculate_rpm(int32_t Z, int32_t L, int32_t M, int32_t N);
//void calibration_mode();
void manual_mode();
void safe_mode();
void panic_mode();
void calibration_mode();
void yaw_control_mode();
void full_control_mode();
void height_control_mode();
void height_control_mode_end(void);
void end_mode(void);
//void check_connection();
//void process_input();

//state pointer
//void (*statefunc)();
uint8_t statefunc;
//void (*setmode)();

//variable to hold current mode
char cur_mode;
char prev_mode;

//p controller value
int p;
int p1;
int p2;
int p3;

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
int32_t lift_force;
int32_t roll_moment;
int32_t pitch_moment;
int32_t yaw_moment;

//dc offset of gyro sensor
int16_t sp_off;
int16_t sq_off;
int16_t sr_off;
int16_t phi_off;
int16_t theta_off;

int32_t sp_off_;
int32_t sq_off_;
int32_t sr_off_;
int32_t phi_off_;
int32_t theta_off_;

//counters to take care of exiting when communication breaks down
extern uint32_t time_latest_packet_us, cur_time_us;

//flags indicating that there is still connection and battery
bool connection;
bool battery;
