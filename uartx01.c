extern void PUT32 ( unsigned int, unsigned int );
extern unsigned int GET32 ( unsigned int );
extern void dummy ( unsigned int );

#define PIN_CLR(pin) PUT32(GPCLR0, 1<<pin)
#define PIN_SET(pin) PUT32(GPSET0, 1<<pin)

/* GPIO control registers */
#define GPFSEL0     0x20200000
#define GPFSEL1     0x20200004
#define GPSET0      0x2020001C
#define GPCLR0      0x20200028
#define GPPUD       0x20200094
#define GPPUDCLK0   0x20200098

/* GPIO edge and leve detect registers */
#define GPEDS0 	0x20200040
#define GPEDS1 	0x20200044
#define GPLEV0	0x20200034
#define GPREN0  0x2020004C
#define GPREN1  0x20200050
#define GPFEN0  0x20200058

/* UART0 registers */
#define UART0_BASE   0x20201000
#define UART0_DR     (UART0_BASE+0x00)
#define UART0_RSRECR (UART0_BASE+0x04)
#define UART0_FR     (UART0_BASE+0x18)
#define UART0_ILPR   (UART0_BASE+0x20)
#define UART0_IBRD   (UART0_BASE+0x24)
#define UART0_FBRD   (UART0_BASE+0x28)
#define UART0_LCRH   (UART0_BASE+0x2C)
#define UART0_CR     (UART0_BASE+0x30)
#define UART0_IFLS   (UART0_BASE+0x34)
#define UART0_IMSC   (UART0_BASE+0x38)
#define UART0_RIS    (UART0_BASE+0x3C)
#define UART0_MIS    (UART0_BASE+0x40)
#define UART0_ICR    (UART0_BASE+0x44)
#define UART0_DMACR  (UART0_BASE+0x48)
#define UART0_ITCR   (UART0_BASE+0x80)
#define UART0_ITIP   (UART0_BASE+0x84)
#define UART0_ITOP   (UART0_BASE+0x88)
#define UART0_TDR    (UART0_BASE+0x8C)

/* UART flags */
#define RX_FIFO_EMPTY   0x10
#define TX_FIFO_FULL    0x20

/* IRQ registers */
#define IRQ_BASIC 0x2000B200
#define IRQ_PEND1 0x2000B204
#define IRQ_PEND2 0x2000B208
#define IRQ_FIQ_CONTROL 0x2000B210
#define IRQ_ENABLE1 0x2000B210
#define IRQ_ENABLE2 0x2000B214
#define IRQ_ENABLE_BASIC 0x2000B218
#define IRQ_DISABLE1 0x2000B21C
#define IRQ_DISABLE2 0x2000B220
#define IRQ_DISABLE_BASIC 0x2000B224

/* Timer register */
#define TIMER_CLO 0x20003004

/* PWM registers */
#define PWM_BASE    0x2020C000
#define CLOCK_BASE  0x20101000
#define PWMCLK_CNTL (CLOCK_BASE + 0xA0)
#define PWMCLK_DIV  (CLOCK_BASE + 0xA4)
#define PWM_CTL     (PWM_BASE + 0)
#define PWM_RNG1    (PWM_BASE + 0x10)
#define PWM_DAT1    (PWM_BASE + 0x14)

/* I2C registers */
//#define BSC0_BASE 0x20205000 //BSC0 base
#define BSC0_BASE 0x20804000 // BSC1 base

#define BSC0_C      (BSC0_BASE + 0x00)
#define BSC0_S      (BSC0_BASE + 0x04)
#define BSC0_DLEN   (BSC0_BASE + 0x08)
#define BSC0_A      (BSC0_BASE + 0x0C)
#define BSC0_FIFO   (BSC0_BASE + 0x10)

/* I2C flags */
#define BSC_C_I2CEN  (1 << 15)
#define BSC_C_INTR   (1 << 10)
#define BSC_C_INTT   (1 << 9)
#define BSC_C_INTD   (1 << 8)
#define BSC_C_ST     (1 << 7)
#define BSC_C_CLEAR  (1 << 4)
#define BSC_C_READ    1

#define START_READ    BSC_C_I2CEN|BSC_C_ST|BSC_C_CLEAR|BSC_C_READ
#define START_WRITE   BSC_C_I2CEN|BSC_C_ST

#define BSC_S_CLKT   (1 << 9)
#define BSC_S_ERR    (1 << 8)
#define BSC_S_RXF    (1 << 7)
#define BSC_S_TXE    (1 << 6)
#define BSC_S_RXD    (1 << 5)
#define BSC_S_TXD    (1 << 4)
#define BSC_S_RXR    (1 << 3)
#define BSC_S_TXW    (1 << 2)
#define BSC_S_DONE   (1 << 1)
#define BSC_S_TA     1

#define CLEAR_STATUS    BSC_S_CLKT|BSC_S_ERR|BSC_S_DONE


// UART documentation
//GPIO14  TXD0 and TXD1
//GPIO15  RXD0 and RXD1
//alt function 5 for uart1
//alt function 0 for uart0

//(3000000 / (16 * 115200) = 1.627
//(0.627*64)+0.5 = 40
//int 1 frac 40

//------------------------------------------------------------------------
// UART output functions
//------------------------------------------------------------------------
void uart_putc(unsigned int c){
  while(1)
    {
      if((GET32(UART0_FR)&TX_FIFO_FULL)==0) break;
    }
  PUT32(UART0_DR,c);
}
//------------------------------------------------------------------------
unsigned int strlen(const char *str){
  unsigned int i;
  for(i=0; i<255 && str[i]!='\x0'; i++)
    ;
  return i;
}
//------------------------------------------------------------------------
void puts(const char *str){
  unsigned int i, n = strlen(str);
  for(i = 0; i < n; uart_putc(str[i++]));
}
//------------------------------------------------------------------------
void hexstrings(unsigned int d){
  //unsigned int ra;
  unsigned int rb;
  unsigned int rc;

  rb=32;
  while(1)
    {
      rb-=4;
      rc=(d>>rb)&0xF;
      if(rc>9) rc+=0x37; else rc+=0x30;
      uart_putc(rc);
      if(rb==0) break;
    }
  uart_putc(0x20);
}
//------------------------------------------------------------------------
void hexstring(unsigned int d){
  hexstrings(d);
  uart_putc(0x0D);
  uart_putc(0x0A);
}
//------------------------------------------------------------------------
// UART input functions
//------------------------------------------------------------------------
unsigned int uart_getc ( void )
{
  while(1)
    {
      if((GET32(UART0_FR)&RX_FIFO_EMPTY)==0) break;
    }
  return(GET32(UART0_DR));
}
//------------------------------------------------------------------------
// Sleep and wait functions
//------------------------------------------------------------------------
void usleep( unsigned int ms )
{
  unsigned int t0 = GET32(TIMER_CLO);
  //assuming 1Mhz clock
  while (GET32(TIMER_CLO) < t0 + 1000*ms) {}
}
//------------------------------------------------------------------------
void wait_i2c_done() {
  //Wait till done, let's use a timeout just in case
  int timeout = 20;
  while((!(GET32(BSC0_S) & BSC_S_DONE)) && --timeout) {
    usleep(50);
  }
  if(timeout == 0)
    puts("i2c timeout\n\r");
}
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
void pwm_init(void){
  //Kill clock
  PUT32(PWMCLK_CNTL, 0x5A000000 | (1<<5));
  usleep(10);
  while (GET32(PWMCLK_CNTL) & 0x00000080) {}
  //This should make the clock frequency approximately 10Khz
  PUT32(PWMCLK_DIV, 0x5A000000 | (1900 << 12));
  //source=osc and enable clock
  PUT32(PWMCLK_CNTL, 0x5A000000 | (1<<4) | 1);
  while (GET32(PWMCLK_CNTL) & 0x00000080) {}
  //disable PWM & start from scratch
  PUT32(PWM_CTL, 0);
  usleep(10);
  //100 PWM clock ticks per PWM period => 100 PWM cycles per second
  PUT32(PWM_RNG1, 100);
  usleep(10);
  //set duty cycle to 50%
  PUT32(PWM_DAT1, 75);
  usleep(10);
  //start PWM1
  PUT32(PWM_CTL, GET32(PWM_CTL) | 1);
}
//------------------------------------------------------------------------
void uart_init ( void )
{
  PUT32(UART0_ICR,0x7FF);
  PUT32(UART0_IBRD,1);
  PUT32(UART0_FBRD,40);
  PUT32(UART0_LCRH,0x70);
  PUT32(UART0_CR,0x301);
}
//------------------------------------------------------------------------
// PWM duty cycle control
//------------------------------------------------------------------------
void pwm_set(unsigned int duty_cycle)
{
  PUT32(PWM_DAT1, duty_cycle);
  usleep(10);
  PUT32(PWM_CTL, GET32(PWM_CTL) | 1);
}
   

//------------------------------------------------------------------------
void c_irq_handler()
{
  if (GET32(GPEDS0)&(1<<18))
    {
      //        counter++;
      PUT32(GPEDS0, 1<<18);
      if (GET32(GPLEV0)&(1<<18))
	PUT32(GPSET0, 1<<17);
      else
	PUT32(GPCLR0, 1<<17);
    }
}
//------------------------------------------------------------------------
int notmain ( unsigned int earlypc )
{
  unsigned int ra, glitch_counter, l0 = 0, l1 = 0;
  unsigned int t0, t1, i, t[5000], a[5000];
  unsigned int step_counter, angle, magnitude;
  char output_buf[100];

  /* clear the pins to stop the motor from running */
  PIN_CLR(7);
  PIN_CLR(8);

  gpio_init();
  pwm_init();
  uart_init();
  hexstring(0x87654321);
  hexstring(&t[0]);
  
  //PUT32(IRQ_ENABLE2, 0);
  //PUT32(IRQ_ENABLE1, 0);
  //PUT32(IRQ_ENABLE2, 1<<(49-32));
  //enable_irq();

    
  //probably a better way to flush the rx fifo.  depending on if and
  //which bootloader you used you might have some stuff show up in the
  //rx fifo.
  while(1)
    {
      if(GET32(UART0_FR)&0x10) break;
      GET32(UART0_DR);
    }

  while(1) {
    puts("> ");
    ra=uart_getc();
    switch(ra) {
    case 'p':  
      /*if (GET32(GPEDS0)&(1<<18))
	{
	counter++;
	PUT32(GPEDS0, 1<<18);
	}
      */
      puts("step_counter="); hexstring(step_counter); puts("\n\r");
      puts("glitch_counter="); hexstring(glitch_counter);
      puts("\n\r");
      break;
      // Record edge transition time stamps for 500 transitions
    case 'm':
      puts("start\n\r");
      i = 0;
      step_counter = 0;
      glitch_counter = 0;
      t0 = GET32(TIMER_CLO);
      l0 = 0;
      while(1) {
	if (GET32(GPEDS0)&(1<<17)) {
	  PUT32(GPEDS0, 1<<17);
	  t[i++] = GET32(TIMER_CLO);
	  if(i>0 && (t[i]-t[i-1])<50) {i--; continue;}
	  if (l0 == 0) {
	    PUT32(GPFEN0, GET32(GPFEN0)|(1<<17));
	    PUT32(GPREN0, GET32(GPREN0)&(~(1<<17)));
	  }
	  else {
	    PUT32(GPREN0, GET32(GPREN0)|(1<<17));
	    PUT32(GPFEN0, GET32(GPFEN0)&(~(1<<17)));
	  }   
	  l0 = !l0;
	  if (i == 500) break;                    
	}
      }
      puts("done\n\r");
      hexstring(i);
      hexstring(glitch_counter);
      break;
    case 'L':
      while(1) {
	hexstring(GET32(GPLEV0)&(1<<17));
      }
      break;
      // Record the inter edge transition times for 400 transitions
    case 'l':
      i = 1;
      a[0] = GET32(TIMER_CLO);
      while(1) {
	if (GET32(GPEDS0)&(1<<17)) {
	  PUT32(GPEDS0, 1<<17);
	  a[i]=GET32(TIMER_CLO);
	  if((a[i]-a[i-1])>50)
	    i++;
	}
	if(i>=400) break;
      }
      for(i=0;i<400-1;i++) hexstring(a[i+1]-a[i]);
      break;
      // Set PWM duty cycle
    case '1': pwm_set(10); puts("duty_cycle=10\n\r"); break;
    case '5': pwm_set(50); puts("duty_cycle=50\n\r"); break;
    case '7': pwm_set(70); puts("duty_cycle=70\n\r"); break;
    case '8': pwm_set(80); puts("duty_cycle=80\n\r"); break;
    case '0': pwm_set(100); puts("duty_cycle=100\n\r"); break;
    case 'g': PIN_SET(7);
      puts("start\n\r");
      i = 0;
      step_counter = 0;
      glitch_counter = 0;
      t0 = GET32(TIMER_CLO);
      l0 = 0;
      while(1) {
	if (GET32(GPEDS0)&(1<<17)) {
	  PUT32(GPEDS0, 1<<17);
	  t[i++] = GET32(TIMER_CLO);
	  if(i>0 && (t[i]-t[i-1])<50) {i--; continue;}
	  if (l0 == 0) {
	    PUT32(GPFEN0, GET32(GPFEN0)|(1<<17));
	    PUT32(GPREN0, GET32(GPREN0)&(~(1<<17)));
	  }
	  else {
	    PUT32(GPREN0, GET32(GPREN0)|(1<<17));
	    PUT32(GPFEN0, GET32(GPFEN0)&(~(1<<17)));
	  }
	  l0 = !l0;
	  if (i == 500) break;
	}
      }
      puts("done\n\r");
      hexstring(i);
      hexstring(glitch_counter); 
      PIN_CLR(7);
      break;
    case 'G': PIN_SET(8);
      usleep(100);
      PIN_CLR(8);
      break;
      /*
      // SPI test
      case 's': 
      spi_begin();
      i = spi_transfer(i);
      spi_end();
      hexstring(i);
      break;
      */
      // I2C test
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
