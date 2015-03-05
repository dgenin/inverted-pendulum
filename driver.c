#include "dev_mem_map.h"
#include "uart.h"
#include "pwm.h"
#include "i2c.h"

/* Global variables */
int dir = 0; //Motor direction
int power = 0; //Motor power
int v = 0; //Current carriage speed
volatile int x = 0; //Current position
volatile int t0 = 0, t = 0; //Previous and current tick times


//------------------------------------------------------------------------
// Initialization functions
//------------------------------------------------------------------------
void gpio_init (void) {
  unsigned int ra;

  PUT32(UART0_CR,0);

  ra = GET32(GPFSEL1);
  //Serial port
  ra &= ~(7<<12); //gpio14
  ra |= 4<<12;    //alt0
  ra &=~ (7<<15); //gpio15
  ra |= 4<<15;    //alt0
  ra &= ~(7<<24); //gpio18
  ra |= 2<<24; //gpio18 set to alt5 = pwm
  ra&=~(7<<21); //gpio17 set to input
  //ra|=1<<21;    //output

  PUT32(GPFSEL1,ra);

  ra=GET32(GPFSEL0);
  ra &= ~(7<<21);
  ra |= 1<<21; // gpio7 set to output
  ra &= ~(7<<24);
  ra |= 1<<24; // gpio8 set to output
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
  t0 = t;
  t = GET32(TIMER_CLO);
  x+=dir;
  PUT32(GPEDS0, 1<<17);
}
//------------------------------------------------------------------------
int notmain ( unsigned int earlypc )
{
  unsigned int ra;
  unsigned int i, j;
  unsigned int angle, magnitude;
  char output_buf[100];

  /* Set initial position to 0 */
  x=0;
  dir = 1;

  /* clear the pins to stop the motor from running */
  PIN_CLR(7);
  PIN_CLR(8);

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
  
  while(1) {
    puts("> ");
    ra=uart_getc();
    switch(ra) {
    case 'p':
      puts("Enter pwm duty cycle 0-100: ");
      power = uart_read_int();
      if (power > 100)
	puts("Invalid duty cycle.\n\r");
      else
	pwm_set(power);
      // Move right 100 ticks
    case 'g': 
      PIN_SET(7);
      dir = 1;
      puts("start\n\r");
      PUT32(GPREN0, GET32(GPREN0)|(1<<17));
      i=x+100;
      j=0;
      while(x<i) {
	  j++;
	  usleep(1);
	  if (j>100) break;
      }
      puts("done\n\r");
      hexstring(i);
      hexstring(x); 
      PIN_CLR(7);
      break;
      // Move left 100 ticks
    case 'G':
      PIN_SET(8);
      dir = -1;
      puts("start\n\rw");
      PUT32(GPREN0, GET32(GPREN0)|(1<<17));
      i=x-100;
      j=0;
      while(x>i) {
	  j++;
	  usleep(1);
	  if (j>100) break;
      }
      puts("done\n\r");
      hexstring(i);
      hexstring(x); 
      PIN_CLR(8);
      break;
      // I2C commands
    case 'i':
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
      break;
    case 'I':
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
    default:
      puts("unrecognized command : "); uart_putc(ra);
      puts("\n\r");
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
