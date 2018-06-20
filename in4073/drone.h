void calculate_rpm(int32_t Z, int32_t L, int32_t M, int32_t N);

//state pointer
uint8_t statefunc;

//variable to hold current mode
char cur_mode;
char prev_mode;

//p controller value
int16_t p;
int16_t p1;
int16_t p2;
int16_t p3;

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
//extern uint32_t time_latest_packet_us, cur_time_us;

//flags indicating that there is still connection and battery
bool connection;
bool battery;
