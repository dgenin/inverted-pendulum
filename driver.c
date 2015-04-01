#include "dev_mem_map.h"
#include "uart.h"
#include "pwm.h"
#include "i2c.h"

#define NOW_TIME() GET32(TIMER_CLO)

#define T_BUFFER_SIZE 10
#define STALL_TIMEOUT 250000 //in microseonds

/* Global variables */
int dir = 0; //Motor direction
int power = 0; //Motor power
int v = 0; //Current carriage speed

/* Globals updated in the tick ISR */
volatile int x = 0; //Current position in units of ticks ~ .005"
volatile unsigned int t[T_BUFFER_SIZE]; //Ring buffer for tick times
volatile unsigned int t_head;

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
  ra &= ~(7<<0);
  /* I2C I/O pins */
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
  /* Increament position according to direction of motion */
  x+=dir;
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
  if (duty_cycle > 100 || duty_cycle < 0)
    puts("Invalid duty cycle.\n\r");
  else
    pwm_set(duty_cycle);
}
//------------------------------------------------------------------------
void motor_run(int direction)
{
  dir = direction;
  if (1 == dir)
    PIN_SET(7);
  else if (-1 == dir)
    PIN_SET(8);
}
//------------------------------------------------------------------------
void motor_stop()
{
  PIN_CLR(7);
  PIN_CLR(8);
  dir = 0;
}
//------------------------------------------------------------------------
void move_to_limit(int direction)
{
  unsigned start_head;

  start_head = t_head;
  set_motor_voltage(70);
  motor_run(direction);
  
  while(t_head == start_head)
    ;
  while(((NOW_TIME() - t[t_head]) < STALL_TIMEOUT) || ((NOW_TIME() - t[t_head]) < STALL_TIMEOUT))
    ;
  motor_stop();
  hexstring(x);
  hexstring(t[t_head]);
  hexstring(NOW_TIME());
}
//------------------------------------------------------------------------
void move(int direction, int ticks)
{
  int i,j;

  puts("start\n\r");
  i=x+direction*ticks;
  j=0;
  motor_run(direction);
  while((direction*(x-i))<0) {
    if((NOW_TIME() - t[t_head]) > STALL_TIMEOUT) break;
    j++;
    usleep(1);
    //if (j>ticks) break;
  }
  motor_stop();
  puts("done\n\r");
  hexstring(i);
  hexstring(x);
}
//------------------------------------------------------------------------
void reset_x()
{
  x=0;
}
//------------------------------------------------------------------------
void i2c1()
{
  while(GET32(BSC0_S) & BSC_S_RXD) {
    GET32(BSC0_FIFO);
  };
  puts("Starting i2c test\n\r");
  puts("Initial i2c status: "); hexstring(GET32(BSC0_S));
  puts("\n\r");
  PUT32(BSC0_S, CLEAR_STATUS); // Reset status bits (see #define)
  PUT32(BSC0_A, 1<<6);
  PUT32(BSC0_DLEN, 1);
  PUT32(BSC0_FIFO, 0xFA);
                
  puts("I2C status after reset: "); hexstring(GET32(BSC0_S));

  PUT32(BSC0_C, START_WRITE);    // Start Write (see #define)
  wait_i2c_done();

  puts("Finished writing\n\r");
  puts("I2C status: ");
  hexstring(GET32(BSC0_S));

  PUT32(BSC0_DLEN, 1);
  PUT32(BSC0_S, CLEAR_STATUS); // Reset status bits (see #define)
  PUT32(BSC0_C, START_READ);    // Start Read after clearing FIFO (see #define)
  wait_i2c_done();

  puts("Read: "); hexstring(GET32(BSC0_FIFO));
}
//------------------------------------------------------------------------
void i2c2()
{
  unsigned int angle, magnitude;

  while(GET32(BSC0_S) & BSC_S_RXD) {
    GET32(BSC0_FIFO);
  };
  PUT32(BSC0_A, 1<<6);
  PUT32(BSC0_DLEN, 1);
  puts("Reading magnet angle position in a loop\n\rUse 'q' to interrupt\n\r");
  while(1){
    // Read angle
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0xFB);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    angle = (GET32(BSC0_FIFO)) << 8; //&0xFF) << 6;
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0xFC);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    angle |= (GET32(BSC0_FIFO)); //&0x3F);
    hexstring(angle);
    //Read magnitude
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0xFF);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    magnitude = (GET32(BSC0_FIFO)) << 8;//&0xFF) << 6;
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_FIFO, 0x00);
    PUT32(BSC0_C, START_WRITE);
    wait_i2c_done();
    PUT32(BSC0_S, CLEAR_STATUS);
    PUT32(BSC0_C, START_READ);
    wait_i2c_done();
    magnitude |= (GET32(BSC0_FIFO)); //&0x3F);
    hexstring(magnitude);
    puts("\n\r");
    usleep(200);
  }
}
//------------------------------------------------------------------------
int notmain ( unsigned int earlypc )
{
  int ra;
  unsigned int i, j;
  unsigned int v;

  /* clear the pins to stop the motor from running */
  PIN_CLR(7);
  PIN_CLR(8);

  /* Set initial position to 0 */
  x=0;
  /* Set initial direction to 0 (stopped) */
  dir = 0;
  /* Clear time ring buffer */
  t_head = 0;
  for(i=0; i<T_BUFFER_SIZE; i++)
    t[i] = 0;

  gpio_init();
  pwm_init();
  uart_init();
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
      dir = 0;
      set_motor_voltage(0);
      puts("stall detected\n\r");
    }
  puts("\n\r> ");

  while(1) {
    ra=uart_getc_nb();
    if(ra>=0)
      switch(ra) {
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
	v = uart_read_int();
	set_motor_voltage(v);
	break;
	// Move right 100 ticks
      case 'g': move(1, 100);
	break;
	// Move left 100 ticks
      case 'G': move(-1, 100);
	break;
	// I2C commands
      case 'i': i2c1();
	break;
      case 'I': i2c2();
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
