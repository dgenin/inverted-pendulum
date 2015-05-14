#include "dev_mem_map.h"
#include "uart.h"
#include "pwm.h"
#include "i2c.h"

#define NOW_TIME() GET32(TIMER_CLO)

#define T_BUFFER_SIZE 10
#define X_BUFFER_SIZE 10
#define STALL_TIMEOUT 250000 //in microseonds
#define CENTER 0x5D00

/* Global variables */
int control_dir = 0; //Motor direction as determined by the control pins
                     //Should only be modified in motor_run() and motor_stop()
unsigned int volts = 0; //Motor power
int speed = 0; //Current carriage speed
int limit = 0; //Half length of the track in ticks, center of the track is 0
unsigned int p1,p2;

/* Globals updated in the tick ISR */
volatile int x; //Current position ring buffer in units of ticks ~ .005"
volatile unsigned int t[T_BUFFER_SIZE]; //Ring buffer for tick times
volatile unsigned int t_head; //Current head of the ring buffer
volatile int carriage_dir = 0; //Carriage direction as seen by the sensor

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

  //enable pull down for gpio 17, 18, 6, 7    
  PUT32(GPPUD,1);
  for(ra=0;ra<150;ra++) dummy(ra);
  PUT32(GPPUDCLK0,(1<<18)|(1<<17)|(1<<6)|(1<<7));
  for(ra=0;ra<150;ra++) dummy(ra);
  PUT32(GPPUDCLK0,0);

  //enable rising edge detection
  //on gpio 17
  PUT32(GPREN0, (1<<17));
  //PUT32(GPFEN0, (1<<18)|(1<<17));
  //PUT32(GPEDS0, (1<<17));
}
   

//------------------------------------------------------------------------
void c_irq_handler()
{
  /* Record the current tick time in tick time buffer */
  t_head = (t_head + 1) % T_BUFFER_SIZE;
  t[t_head] = GET32(TIMER_CLO);
  /* Detect direction of the carriage motion based on the
   * second channel's level 
   */
  carriage_dir = (GET32(GPLEV0)&(1<<9)) ? 1 : -1;
  /* Increament position according to direction of motion */
  x += carriage_dir;
  /* Clear edge detect interrupt flag.
   * Should be last to avoid (the very unlikely) ISR reentry.
   */
  PUT32(GPEDS0, 1<<17);
}
//------------------------------------------------------------------------
/* Process command line input*/
void proccess_cmd(unsigned int cmd)
{
}
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
/* Pins 7 and 8 should only be manipulated in motor_run and motor_stop */
void motor_run(int direction)
{
  if (control_dir == direction)
    return;
  if (1 == direction)
    {
      control_dir = 1;
      PIN_CLR(7);
      PIN_SET(8);
    }
  else if (-1 == direction)
    {
      control_dir = -1;
      PIN_CLR(8);
      PIN_SET(7);
    }
}
//------------------------------------------------------------------------
void motor_stop()
{
  PIN_SET(7);
  PIN_SET(8);
  control_dir = 0;
}
void motor_stop1()
{
  PIN_CLR(7);
  PIN_CLR(8);
  control_dir = 0;
}
//------------------------------------------------------------------------
void move_to_limit(int direction)
{
  unsigned start_head;

  start_head = t_head;
  set_motor_voltage(75);
  motor_run(direction);
  
  while(t_head == start_head)
    ;
  while(((NOW_TIME() - t[t_head]) < STALL_TIMEOUT) || ((NOW_TIME() - t[t_head]) < STALL_TIMEOUT))
    ;
  motor_stop();
  hexstring(x);
  //hexstring(t[t_head]);
  //hexstring(NOW_TIME());
}
//------------------------------------------------------------------------
void move(int direction, int ticks)
{
  int i,j;

  //puts("move\n\r");
  i = x + direction*ticks;
  j = 0;
  motor_run(direction);
  while((direction*(x-i))<0) {
    //TODO: Figure out why the stall check fails closed like a valve
    /*if(!(((NOW_TIME() - t[t_head]) < STALL_TIMEOUT) || ((NOW_TIME() - t[t_head]) < STALL_TIMEOUT)))
      {
	puts("stalled\n\r");
	break;
      }
    */
    j++;
    usleep(1);
    //if (j>ticks) break;
  }
  motor_stop();
  //puts("done\n\r");
  //hexstring(i);
  //hexstring(x);
}
//------------------------------------------------------------------------
void move_to(int position)
{
  if (x>position)
    move(-1, x-position);
  else
    move(1, position-x);
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
  move_to_limit(-1);
  reset_x();
  move_to_limit(1);
  limit = x/2;
  hexstring(limit);
  move(-1, limit);
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
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    angle = (GET32(BSC0_FIFO)) << 8;//&0xFF) << 6;
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0x00);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    angle |= (GET32(BSC0_FIFO)); //&0x3F);
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
      move(1 - 2*(i%2), 200);
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
//------------------------------------------------------------------------
void balance_debug()
{
  //Smaller angle values left of CENTER, i.e., angle increases in the clockwise direction
  unsigned int angle;
  int dir;
  dir = 0;

  while(1)
    {
      angle = read_angle();
      if (angle > 0x9000)
	break;
      if (angle < CENTER)
	{
	  volts = (CENTER-angle);//p1;
	  volts = volts;///p2;
	  dir = 1;
	}
      else
	{
	  volts = (angle-CENTER);///p1;
	  volts = volts;//p2;
	  dir = -1;
	}
      hexstring(volts);
      hexstring(dir);
      usleep(500);
    }
}

void decompose_angle(unsigned int angle, int *abs_dev, int *dir)
{
  if (angle < CENTER)
    {
      *abs_dev = (CENTER-angle);
      *dir = 1;
    }
  else
    {
      *abs_dev = (angle-CENTER);
      *dir = -1;
    }
}  

void balance()
{
  //Smaller angle values left of CENTER, i.e., angle increases in the clockwise direction
  unsigned int angle, angle_prev;
  unsigned int a_prev, a;
  int dir, dir_prev = 0;
  //  int correction, p_error, last_p_error, dir, d_error, i_error;
  
  set_motor_voltage(75);
  move_to(0);
  uart_getc();
  // For testing without running the motor
  //reset_x();
  //limit  = 0;

  /* last_p_error = 0; */
  /* i_error = 0; */
  /* d_error = 0; */
  /* correction = 0; */

  decompose_angle(read_angle()+x/0x80, &a_prev, &dir_prev);
  while(1)
    {
      angle = read_angle();
      /* p_error = angle - CENTER; */
      /* d_error = last_p_error - p_error; */
      /* i_error += p_error;  */
      /* correction = p_error + d_error + i_error; */
      /* control_dir = (correction > 0) ? -1 : 1; */
      /* last_p_error = p_error; */
      /* set_motor_voltage(-dir*correction/5); */
      /* motor_run(dir); */

      angle += x/0x60;
      decompose_angle(angle, &a, &dir);
      if (a > 0x750)
	{
	  motor_stop();
	  puts("Reached critical angle\r\n");
	  break;
	} 
      //compensate for uncertainty in the CENTER angle
      //if the CENTER has not bee crossed and angle is getting closer to CENTER
      // and angle is small continue doing whatever we were doing
      //if ((dir_prev == dir) && (a < a_prev) && (a < 0x20))
      //continue;
      volts = (a*a)/(0x190000/40)+66;
      //volts = abs_rel_angle/(0xef0/70)+70;
      hexstring(a);
      hexstring(a_prev);
      hexstring(dir);
      hexstring(dir_prev);
      hexstring(volts);
      puts("-----------------\r\n");
      set_motor_voltage(volts);
      motor_run(dir);

      if ((x > limit) || (x < -limit))
	{
	  motor_stop();
	  puts("hit limit\n\r");
	  return;
	}
      usleep(1);
      a_prev = a;
      dir_prev = dir;
    }
}
//------------------------------------------------------------------------
int notmain ( unsigned int earlypc )
{
  int ra;
  unsigned int i, j;

  /* Reset motor control pins and zero out control_dir */
  motor_stop();

  /* Clear time and position ring buffers */
  t_head = 0;
  for(i=0; i<T_BUFFER_SIZE; i++)
    t[i] = 0;

  gpio_init();
  pwm_init();
  uart_init();
  i2c_init();
  hexstring(0x87654321);
  
  //PUT32(IRQ_ENABLE2, 0);
  //PUT32(IRQ_ENABLE1, 0);
  /* Enable IRQ 49 for edge detection on gpio 17 */ 
  PUT32(IRQ_ENABLE2, 1<<(49-32));
  enable_irq();
    
  //probably a better way to flush the rx fifo.  depending on if and
  //which bootloader you used you might have some stuff show up in the
  //rx fifo.
  uart_flush_rx_fifo();
  if(NOW_TIME() - t[t_head] > 1000)
    {
      motor_stop();
      set_motor_voltage(0);
      puts("stall detected\n\r");
    }
  puts("\n\r> ");

  p1 = 7;
  p2 = 125;

  while(1) {
    ra=uart_getc_nb();
    if(ra>=0)
      switch(ra) {
      case 'c':
	calibrate();
	break;
      case 'd':
	balance_debug();
	break;
      case 'b':
	balance();
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
	puts("unrecognized command : "); uart_putc(ra); hexstrings(ra);
	puts("\n\r> ");
      }            
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
