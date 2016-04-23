#define T_BUFFER_SIZE 100000
#define C_BUFFER_SIZE 50000
#define X_BUFFER_SIZE 10
#define STALL_TIMEOUT 250000 //in microseonds

/* Global variables */
ABROAD int control_dir; //Motor direction as determined by the control pins
                     //Should only be modified in motor_run() and motor_stop()
ABROAD unsigned int volts; //Motor power
ABROAD int speed; //Current carriage speed
ABROAD int limit; //Half length of the track in ticks, center of the track is 0
ABROAD unsigned int p1,p2;
ABROAD int center_bias; //Bias to center setting
ABROAD int lean; // Angle to add to center to keep in the middle of the track
ABROAD int a2_scale; // Used to scale angle ^ 2 to obtain voltage in balance
ABROAD int a2_offset; // Used to offset angle ^ 2/a2_scale to obtain voltage in balance
ABROAD int ah; //Used to determine at what lean angle motor direction should reversed

/* Globals updated in the tick ISR */
ABROAD volatile int x; //Current position ring buffer in units of ticks ~ .006"
//ABROAD volatile unsigned int t[T_BUFFER_SIZE]; //Ring buffer for tick times
ABROAD volatile unsigned int encoder[C_BUFFER_SIZE]; //Ring buffer for motor voltage
//ABROAD volatile unsigned int t_head; //Current head of the ring buffer
ABROAD volatile unsigned int last_tick; //Timestamp of the last encoder tick
ABROAD volatile unsigned int encoder_head; //Current head of the encoder buffer
ABROAD volatile int carriage_dir; //Carriage direction as seen by the sensor
ABROAD volatile int target_speed; //Reciprocal of the target carriage speed in ticks per stripe 
ABROAD volatile int v; //Observed velocity of the carriage in ticks per stripe
ABROAD volatile unsigned int tick_error_count;

void set_motor_voltage(unsigned int duty_cycle);
void motor_run(int direction);
void motor_stop();
void motor_stop1();
void move_to_limit(int direction);
void move(int direction, int ticks);
void move_to(int position);



