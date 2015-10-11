#define T_BUFFER_SIZE 100000
#define X_BUFFER_SIZE 10
#define STALL_TIMEOUT 250000 //in microseonds

/* Global variables */
ABROAD int control_dir; //Motor direction as determined by the control pins
                     //Should only be modified in motor_run() and motor_stop()
ABROAD unsigned int volts; //Motor power
ABROAD int speed; //Current carriage speed
ABROAD int limit; //Half length of the track in ticks, center of the track is 0
ABROAD unsigned int p1,p2;

/* Globals updated in the tick ISR */
ABROAD volatile int x; //Current position ring buffer in units of ticks ~ .006"
ABROAD volatile unsigned int t[T_BUFFER_SIZE]; //Ring buffer for tick times
ABROAD volatile unsigned int t_head; //Current head of the ring buffer
ABROAD volatile int carriage_dir; //Carriage direction as seen by the sensor
ABROAD volatile int target_speed; //Reciprocal of the target carriage speed in ticks per stripe 
ABROAD volatile int v; //Observed velocity of the carriage in ticks per stripe

void set_motor_voltage(unsigned int duty_cycle);
void motor_run(int direction);
void motor_stop();
void motor_stop1();
void move_to_limit(int direction);
void move(int direction, int ticks);
void move_to(int position);



