#include "dev_mem_map.h"
#include "uart.h"
#include "pwm.h"
#include "i2c.h"
#include "cli.h"
void enable_irq(void);

#define ABROAD extern
#include "driver.h"
#undef ABROAD

// Leaning center to the right to compensate
#define CENTER (5900 + center_bias)
#define TIMER_INTERVAL 100
volatile unsigned int timer_next;
int da[10000], dt[10000];
volatile static unsigned int last_state;

//Hack: couldn't get the ld to link the correct libc function
void __div0(void)
{
}

void raise(void)
{
}

/* Dummy function to stop compiler from optimizing trivial loops */
extern void dummy ( unsigned int );
//------------------------------------------------------------------------
// Initialization functions
//------------------------------------------------------------------------
void gpio_init (void) {
  unsigned int ra;

  PUT32(UART0_CR,0);

  ra = GET32(GPFSEL1);
  /* Serial port I/O pins */ 
  ra &= ~(7<<12); //gpio14
  ra |= 4<<12;    //alt0
  ra &=~ (7<<15); //gpio15
  ra |= 4<<15;    //alt0
  /* PWM pin */
  ra &= ~(7<<24); //gpio18
  ra |= 2<<24; //gpio18 set to alt5 = pwm
  /* Optical encoder input pin */
  ra&=~(7<<21); //gpio17 set to input
  //ra|=1<<21;    //output

  PUT32(GPFSEL1,ra);

  ra=GET32(GPFSEL0);
  /* Motor direction control pins */
  ra &= ~(7<<21);
  ra |= 1<<21; // gpio7 set to output
  ra &= ~(7<<24);
  ra |= 1<<24; // gpio8 set to output
  /* Optical encoder input pin */
  ra &= ~(7<<27); // gpio9 set to input
  /* I2C I/O pins */
  //TODO: remove i2c0 code
  ra &= ~(7<<0);
  ra |= (4<<0); // gpio0 set to alt0: sda0
  ra &= ~(7<<3);
  ra |= (4<<3); // gpio1 set to alt0: scl0
  ra &= ~(7<<6);
  ra |= (4<<6); // gpio2 set to alt0: sda1
  ra &= ~(7<<9);
  ra |= (4<<9); // gpio3 set to alt0: scl1
  PUT32(GPFSEL0,ra);

  //disable pull up/down to gpio 0, 1, 2, 3, 14, 15
  PUT32(GPPUD,0);
  for(ra=0;ra<150;ra++) dummy(ra);
  PUT32(GPPUDCLK0,(1<<14)|(1<<15)|(1<<0)|(1<<1)|(1<<2)|(1<<3));
  for(ra=0;ra<150;ra++) dummy(ra);
  PUT32(GPPUDCLK0,0);

  //enable pull down for gpio 17, 18, 6, 7, 9    
  PUT32(GPPUD,1);
  for(ra=0;ra<150;ra++) dummy(ra);
  PUT32(GPPUDCLK0,(1<<18)|(1<<17)|(1<<6)|(1<<7)|(1<<9));
  for(ra=0;ra<150;ra++) dummy(ra);
  PUT32(GPPUDCLK0,0);

  //enable rising edge detection
  //on gpio 17 & 9
  PUT32(GPFEN0, (1<<17)|(1<<9));
  PUT32(GPREN0, (1<<17)|(1<<9));
  //PUT32(GPFEN0, (1<<18)|(1<<17));
  //PUT32(GPEDS0, (1<<17));
}

void init() {
  /*
    lean = 100
    a2s = 19456
    a2o = 67
    cb = 80
    ah = 20
  */

  x = 0;
  center_bias = -125;
  a2_scale = 3000;
  a2_offset = 72;
  lean = 50;
  ah = 20;
  //For debugging position drift
  encoder_head = 0;
  //Last state of the position state machine
  last_state = 0;
  tick_error_count = 0;
  gpio_init();
  pwm_init();
  uart_init();
  i2c_init();
  cli_init();
}

//------------------------------------------------------------------------
static void position_irq_handler()
{
#if 0
  unsigned int t0, t1;
  unsigned int v_head;
  /* Detect direction of the carriage motion based on the
   * second channel's level 
   */
  v_head = t_head;
  carriage_dir = (GET32(GPLEV0)&(1<<9)) ? -1 : 1;
  //Last time step
  t0 = t[t_head];
  /* Record the current tick time in tick time buffer */
  t_head = (t_head + 1) % T_BUFFER_SIZE;
  t1 = t[t_head] = GET32(TIMER_CLO); 
  /* Increament position according to direction of motion */
  x += carriage_dir;
  /* Clear edge detect interrupt flag.
   * Should be last to avoid (the very unlikely) ISR reentry.
   */
  PUT32(GPEDS0, 1<<17);
#else

  unsigned int curr_state;
  
  last_tick = NOW_TIME();
  curr_state = 0;
  if (GET32(GPEDS0) & (1<<17)) {
    PUT32(GPEDS0, 1<<17);
  };
  if (GET32(GPEDS0) & (1<<9)) {
    PUT32(GPEDS0, 1<<9);
  };
  curr_state = GET32(GPLEV0)&((1<<9)|(1<<17));
  curr_state = ((curr_state&(1<<9))?0:1) | ((curr_state&(1<<17))?0:2);
  switch(last_state){
  case 0 : 
    switch(curr_state){
    case 0 : break;
    case 1 : x++; break; //TODO: Check that the sign is right.
    case 2 : x--; break;
    case 3 : tick_error_count++; break; //TODO: Handle the error case correctly.
    }; break;
  case 1 : switch(curr_state){
    case 1 : break;
    case 3 : x++; break; //TODO: Check that the sign is right.
    case 0 : x--; break;
    case 2 : tick_error_count++; break; //TODO: Handle the error case correctly.
    }; break;
  case 2 : switch(curr_state){
    case 2 : break;
    case 0 : x++; break; //TODO: Check that the sign is right.
    case 3 : x--; break;
    case 1 : tick_error_count++; break; //TODO: Handle the error case correctly.
    }; break;
  case 3 : switch(curr_state){
    case 3 : break;
    case 2 : x++; break; //TODO: Check that the sign is right.
    case 1 : x--; break;
    case 0 : tick_error_count++; break; //TODO: Handle the error case correctly.
    }; break;
  }  
  last_state = curr_state;
#endif
}
//------------------------------------------------------------------------
static void timer_irq_handler()
{
  unsigned int v;
  if (encoder_head < C_BUFFER_SIZE) {
    v = (GET32(GPLEV0)&(1<<9))?0:1;
    v |= (GET32(GPLEV0)&(1<<17))?0:2;
    encoder[encoder_head] = v;
    encoder_head = (encoder_head + 1);
  }
  timer_next = timer_next + TIMER_INTERVAL;
  PUT32(TIMER_C1, timer_next);
  PUT32(TIMER_CS, 2);
}
//------------------------------------------------------------------------
void c_irq_handler()
{
  unsigned int irq_basic, irq_pending1, irq_pending2;
  irq_basic = GET32(IRQ_BASIC);
  if (irq_basic & (1 << 8)) {
    irq_pending1 = GET32(IRQ_PEND1);
    if (irq_pending1 & (1 << 1))
      timer_irq_handler();
  }

  if (irq_basic & (1 << 9)) {
    irq_pending2 = GET32(IRQ_PEND2);
    if (irq_pending2 & (1 << (49 - 32)))
      position_irq_handler();
  }

#if 0
  if(GET32(TIMER_CS)&2)
    timer_irq_handler();
  else
    position_irq_handler();
#endif
}
//------------------------------------------------------------------------
/* Process command line input*/
void proccess_cmd(unsigned int cmd)
{
}
//------------------------------------------------------------------------
/* Set motor voltage */
void set_motor_voltage(unsigned int duty_cycle)
{
  if (duty_cycle > 100)
    duty_cycle = 100;
    else if (duty_cycle < 0)
      duty_cycle = 0;
  pwm_set(duty_cycle);
}
//------------------------------------------------------------------------
void print_x()
{
  puts("current position:");
  hexstring(x);
}
//------------------------------------------------------------------------
void reset_x()
{
  x=0;
}
//------------------------------------------------------------------------
void homing()
{
  move_to_limit(1);
  reset_x();
  move_to_limit(-1);
  limit = -x/2;
  hexstring(limit);
  move(1, limit);
  reset_x();
  limit -= 200;
}
//------------------------------------------------------------------------
unsigned int read_magnitude()
{
  unsigned int magnitude;
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0xFB);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    magnitude = (GET32(BSC0_FIFO)) << 8; //&0xFF) << 6;

    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0xFC);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    magnitude |= (GET32(BSC0_FIFO)); //&0x3F);
    return magnitude;
}
//------------------------------------------------------------------------
unsigned int read_angle()
{
  unsigned int angle;
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0xFF);
    PUT32(BSC0_C, START_WRITE);
    if (wait_i2c_done()) puts("i2c timed out\r\n");
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    if (wait_i2c_done()) puts("i2c timed out\r\n");
    angle = (GET32(BSC0_FIFO)&0xFF) << 6;//&0xFF) << 6;

    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0x00);
    PUT32(BSC0_C, START_WRITE);
    if (wait_i2c_done()) puts("i2c timed out\r\n");
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    if (wait_i2c_done()) puts("i2c timed out\r\n");
    angle |= (GET32(BSC0_FIFO)&0x3F); //&0x3F);

    return angle;
}
//------------------------------------------------------------------------

void calibrate()
{
  int i=0;
  unsigned int t0 = 0;
  for(i=100; i>60; i--)
    {
      set_motor_voltage(i);
      t0 = NOW_TIME();
      //move(1 - 2*(i%2), 200);
      motor_run(1);
      usleep(3000);
      motor_stop();
      puts("\n\r");
      hexstring(i);
      hexstring(NOW_TIME()-t0);
    }
  motor_stop();
}
//------------------------------------------------------------------------
// Current reciprocal velocity in ms/tick
// Assumes that the ring buffer does not wrap around between assignments
/* int dt() */
/* { */
/*   unsigned int now_head, t_now, t_last; */
  
/*   now_head = t_head; */
/*   t_now = t[now_head]; */
/*   t_last = t[(now_head-1)%T_BUFFER_SIZE]; */
/*   return (t_now-t_last)/1000; */
/* } */
/* int last_dt() */
/* { */
/*   unsigned int last_head, t_last, t_lastlast; */
  
/*   last_head = (t_head - 1)%T_BUFFER_SIZE; */
/*   t_last = t[last_head]; */
/*   t_llast = t[(last_head-1)%T_BUFFER_SIZE]; */
/*   return (t_last-t_llast)/1000; */
/* } */

//------------------------------------------------------------------------
//1/v0-1/v1=(v1-v0)/(v1v0)
/* unsigned int accel() */
/* { */
/*   unsigned int now_head, t_now, t_last, t_llast; */
  
/*   now_head = t_head; */
/*   t_now = t[now_head]; */
/*   t_last = t[(now_head-1)%T_BUFFER_SIZE); */
/*   t_llast = t[(now_head-2)%T_BUFFER_SIZE); */
/*   return ((t_now-t_last)-(t_last-t_llast))/1000; */
/* } */
  
//  (us/tick)*(ms/1000us)=ms/tick
//x1=x0+speed*time
//  if (x < x1)
//    set_motor_voltage(;
//  else
//    adfasdf;
// 
// Horizontal velocity of the top of the pendulum. Approximately equal to
// radial velocity times the length of the pendulum for small angles.
//------------------------------------------------------------------------
void decompose_angle(unsigned int angle, int *abs_dev, int *dir)
{
  unsigned int center;
  
  center = CENTER + (x*lean)/2000;

  if (angle < center)
    {
      *abs_dev = (center-angle);
      *dir = 1;
    }
  else
    {
      *abs_dev = (angle-center);
      *dir = -1;
    }
}
//------------------------------------------------------------------------
void balance()
{
  unsigned int angle, angle_prev; //Current and previous raw angle sensor readings
  unsigned int time, time_prev; //Current and last angle sensor readings timestamps
  unsigned int a; //Absolute value of angle relative to vertical position and direction
  int w; //Angular velocity of the pendulum
  int dir; //Direction to run the carriage in
  int i;
  char str[1024];

  //This turns off the speed control loop in the OE ISR 
  target_speed = 0;
  set_motor_voltage(75);
  move_to(0);
  uart_getc();

  //Initialize control loop variables
  time_prev = NOW_TIME();
  angle_prev = read_angle();
  dir = 0;
  i = 0;
  //puts("--------------\n\r");
  //hexstring(&da);
  //hexstring(&dt);
  while(1)
    {
      angle = read_angle();
      time = NOW_TIME();
      w = (0x2000*(int)(angle - angle_prev))/(int)(time - time_prev);
      //da[i % 10000] = angle - angle_prev;
      //dt[i % 10000] = time - time_prev;
      //i++;
      angle_prev = angle;
      time_prev = time;

      //Stop if the angle is to large
      decompose_angle(angle, &a, &dir);
#if 0
      puts("--------------\n\r");
      puts("angle: "); hexstring(angle);
      puts("da:    "); hexstring_signed(angle - angle_prev);
      puts("dt:    "); hexstring(time - time_prev);
      puts("w:     "); hexstring_signed(w);
      puts("a:     "); hexstring(a);
#endif

      //sprintf(str, "%d, %d, %d\r\n", x, dir*a, control_dir*volts);
      //puts(str);
      if(a > 455) // 455 = 10 degrees
	{
	  puts("Reached critical angle\r\n");
	  break;
	}
      if ((x > limit) || (x < -limit))
	{
	  puts("hit limit\n\r");
	  break;
	}

      volts = (a*a)/a2_scale + a2_offset;
      set_motor_voltage(volts);
      if((dir > 0) && (a > ah) && (w < -0x10)) {
	//set_motor_voltage(80);
	motor_run(1);
      }
      if((dir < 0) && (a > ah) && (w > 0x10)) {
	//set_motor_voltage(80);
	motor_run(-1);
      }
      usleep(10);

      if(uart_getc_nb()>=0) break;
    }
  motor_stop();
}
//------------------------------------------------------------------------
/* void balance() */
/* { */
/*   //Smaller angle values right of CENTER, i.e., angle increases in the counter-clockwise direction */
/*   unsigned int angle, angle_prev; */
/*   unsigned int time, time_prev; */
/*   unsigned int a_prev, a; */
/*   int dir, dir_prev = 0; //Pendulum rotation sense +1 -> clockwise, -1 -> counterclockwise */
/*   float w; //Angular velocity of the pendulum */
/*   float v; //Velocity of the carriage */

/*   target_speed = 0; */
/*   set_motor_voltage(75); */
/*   move_to(0); */
/*   uart_getc(); */
/*   // For testing without running the motor */
/*   //reset_x(); */
/*   //limit  = 0; */

/*   //decompose_angle(((int)read_angle())-(x/0x8), &a_prev, &dir_prev); */
/*   //decompose_angle(read_angle(), &a_prev, &dir_prev); */
/*   time = time_prev = NOW_TIME(); */
/*   while(1) */
/*     { */
/*       angle = read_angle(); */
/*       time = NOW_TIME(); */
/*       w = (angle - angle_prev)/(float)(time - time_prev); */
/*       //Might get weird if t buffer gets updated during this computation */
/*       v = 1/(float)(t[t_head] - t[t_head-1]); */
/*       //v = 1/(float) (time - time_prev); */
/*       time_prev = time; */
/*       decompose_angle(angle, &a, &dir); */
/*       if (a > 0x750) */
/* 	{ */
/* 	  motor_stop(); */
/* 	  puts("Reached critical angle\r\n"); */
/* 	  break; */
/* 	}  */
/*       volts = (a*a)/(0x190000/20)+70; */
/*       if a < 0x10 && w >= -0x10 && carriage_dir > 0 */
/*       //volts = (a*a)/(0x190000/40)+66; */
/*       //volts = a/(0xef0/70)+70; */
/*       //hexstring(angle); */
/*       //hexstring(a_prev); */
/*       //hexstring(dir); */
/*       //hexstring(dir_prev); */
/*       //hexstring(volts); */
/*       //puts("-----------------\r\n"); */
/*       set_motor_voltage(volts); */
/*       motor_run(dir); */

/*       if ((x > limit) || (x < -limit)) */
/* 	{ */
/* 	  motor_stop(); */
/* 	  puts("hit limit\n\r"); */
/* 	  return; */
/* 	} */
/*       //usleep(1); */
/*       a_prev = a; */
/*       dir_prev = dir; */
/*     } */
/* } */
//------------------------------------------------------------------------
void speed_test(void)
{
  unsigned int time, time_prev, print_counter;
  int direction;

  puts("in speed test\n");
  direction = 1;
  target_speed = 250;
  motor_run(direction);
  time = time_prev = NOW_TIME();
  print_counter = 0;
  while(1)
    {
     if (x > limit)
       {
       direction = -1;
       time = NOW_TIME();
       time_prev = time;
       }
     if (x < -limit) {
       direction = 1;
       //hexstring(x);
     }
     motor_run(direction);
     //usleep(10);
     //hexstring(x);
     if (print_counter % 1000000)
       hexstring(v);
     print_counter++;

     if(uart_getc_nb()>=0) break;
    }
  target_speed = 0;
  motor_stop();
}
//------------------------------------------------------------------------
int notmain ( unsigned int earlypc )
{
  int ra;
  unsigned int i, j;
  control_dir = 0;
  volts = 0;
  carriage_dir = 0;
  speed = 0;
  limit = 0;
  target_speed = 0;

  /* Reset motor control pins and zero out control_dir */
  motor_stop();

  /* Clear time and position ring buffers */
  last_tick = 0;
  //t_head = 0;
  //for(i=0; i<T_BUFFER_SIZE; i++)
  //  t[i] = 0;

  init();
  hexstring(0x87654321);
  
  hexstring(0x1);

  
  //PUT32(IRQ_ENABLE2, 0);
  //PUT32(IRQ_ENABLE1, 0);
  /* Enable IRQ 49 for edge detection on gpio 17 */ 
  PUT32(IRQ_ENABLE2, (1<<(49-32)));
  PUT32(IRQ_ENABLE1, (1<<1));
  hexstring(0x2);
  timer_next = NOW_TIME() + TIMER_INTERVAL;
  PUT32(TIMER_CS, 2);
  PUT32(TIMER_C1, timer_next);
  enable_irq();
  hexstring(0x3);

  //probably a better way to flush the rx fifo.  depending on if and
  //which bootloader you used you might have some stuff show up in the
  //rx fifo.
  uart_flush_rx_fifo();
  if(NOW_TIME() - last_tick > 1000)
    {
      motor_stop();
      set_motor_voltage(0);
      puts("stall detected\n\r");
    }

  p1 = 7;
  p2 = 125;

  while(1) {
    ra=uart_getc_nb();
    if(ra>=0)
      cli(ra);

#if 0
      switch(ra) {
      case 't':
	{
	  static unsigned int t_old;
	  hexstring(timer_test);
	  timer_test = 0;
	  t_old = timer_test;
	}
	break;
      case 'c':
	calibrate();
	break;
      case 'b':
	balance();
	//speed_test();
	break;
      case 'r':
	reset_x();
	break;
      case 'm':
	move_to_limit(1);
	break;
      case 'M':
	move_to_limit(-1);
	break;
      case 'p':
	puts("Enter pwm duty cycle 0-100: ");
	volts = uart_read_int();
	set_motor_voltage(volts);
	puts("speed set to: ");
	hexstring(volts);
	break;
	// Move right 100 ticks
      case 'g': move(1, 100);
	break;
	// Move left 100 ticks
      case 'G': move(-1, 100);
	break;
      case 'x': print_x();
	break;
	// I2C commands
      case 'i': hexstring(read_angle());
	break;
      case 'h': homing();
	break;
      case '1': puts("Enter p1:");
	p1 = uart_read_int();
	hexstring(p1);
	break;
      case '2': puts("Enter p2:");
	p2 = uart_read_int();
	hexstring(p2);
	break;
      default:
	puts("unrecognized command : ");
	if(ra != '\n' && ra != '\r') {
	  uart_putc(ra);
	  puts(", ");
	}
	puts("hex=");
	hexstrings(ra);
	puts("\n\r> ");
      }
#endif
  }

  return(0);
}
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
//
// Copyright (c) 2012 David Welch dwelch@dwelch.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
//-------------------------------------------------------------------------
