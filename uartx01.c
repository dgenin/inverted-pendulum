#include "spi.h"
//-------------------------------------------------------------------------
//-------------------------------------------------------------------------

extern void PUT32 ( unsigned int, unsigned int );
extern unsigned int GET32 ( unsigned int );
extern void dummy ( unsigned int );

#define GPFSEL0 0x20200000
#define GPFSEL1 0x20200004
#define GPSET0  0x2020001C
#define GPCLR0  0x20200028
#define GPPUD       0x20200094
#define GPPUDCLK0   0x20200098

#define GPEDS0 	0x20200040
#define GPEDS1 	0x20200044
#define GPLEV0	0x20200034
#define GPREN0  0x2020004C
#define GPREN1  0x20200050
#define GPFEN0  0x20200058

//#define CH_A_RISING_EDGE_ON PUT32(GPREN0, GET32(GPREN0) & (1<<17))
//#define CH_B_FALLING_EDGE_ON PUT32(

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

#define TIMER_CLO 0x20003004

#define PWM_BASE 0x2020C000
#define CLOCK_BASE 0x20101000
#define PWMCLK_CNTL (CLOCK_BASE + 0xA0)
#define PWMCLK_DIV  (CLOCK_BASE + 0xA4)
#define PWM_CTL     (PWM_BASE + 0)
#define PWM_RNG1    (PWM_BASE + 0x10)
#define PWM_DAT1    (PWM_BASE + 0x14)

//#define BSC0_BASE 0x20205000 //BSC0 base
#define BSC0_BASE 0x20804000 // BSC1 base

#define BSC0_C      (BSC0_BASE + 0x00)
#define BSC0_S      (BSC0_BASE + 0x04)
#define BSC0_DLEN   (BSC0_BASE + 0x08)
#define BSC0_A      (BSC0_BASE + 0x0C)
#define BSC0_FIFO   (BSC0_BASE + 0x10)

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


//GPIO14  TXD0 and TXD1
//GPIO15  RXD0 and RXD1
//alt function 5 for uart1
//alt function 0 for uart0

//(3000000 / (16 * 115200) = 1.627
//(0.627*64)+0.5 = 40
//int 1 frac 40

void puts( const char *str, unsigned int n );

//------------------------------------------------------------------------
void uart_putc ( unsigned int c )
{
    while(1)
    {
        if((GET32(UART0_FR)&0x20)==0) break;
    }
    PUT32(UART0_DR,c);
}
//------------------------------------------------------------------------
unsigned int uart_getc ( void )
{
    while(1)
    {
        if((GET32(UART0_FR)&0x10)==0) break;
    }
    return(GET32(UART0_DR));
}
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
            puts("i2c timeout\n\r",12);
}
//------------------------------------------------------------------------
void gpio_init ( void )
{
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
    //ra&=~(7<<21); //gpio17 set to input
    //ra|=1<<21;    //output

    PUT32(GPFSEL1,ra);

    ra=GET32(GPFSEL0);
    ra &= ~(7<<21);
    ra |= 1<<21; // gpio7 set to output
    ra &= ~(7<<24);
    ra |= 1<<24; // gpio8 set to output
    ra &= ~(7<<0);
    ra |= (4<<0); // gpio0 set to alt0: sda
    ra &= ~(7<<3);
    ra |= (4<<3); // gpio1 set to alt0: scl
    ra &= ~(7<<6);
    ra |= (4<<6);
    ra &= ~(7<<9);
    ra |= (4<<9);
    PUT32(GPFSEL0,ra);

    //disable pull up/down to gpio 0, 1, 14, 15
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
    //PUT32(GPREN0, (1<<17));
    //PUT32(GPFEN0, (1<<18)|(1<<17));
    //PUT32(GPEDS0, (1<<17));
}
//------------------------------------------------------------------------
void pwm_init( void )
{
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
void pwm_set(unsigned int duty_cycle)
{
    PUT32(PWM_DAT1, duty_cycle);
    usleep(10);
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
void hexstrings ( unsigned int d )
{
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
void hexstring ( unsigned int d )
{
    hexstrings(d);
    uart_putc(0x0D);
    uart_putc(0x0A);
}
//------------------------------------------------------------------------
void puts( const char *str, unsigned int n )
{
    unsigned int i;
    for(i = 0; i < n; uart_putc(str[i++]));
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
    unsigned int tA_0, tA_1, tB_0, tB_1, i, iA, iB, tA[5000], tB[5000], a[5000];
    unsigned int current_state, next_state;
    int step_counter;

    step_counter = 0;
    glitch_counter = 0;
    tB_0 = tA_0 = GET32(TIMER_CLO);
    gpio_init();
    //pwm_init();
    uart_init();
    hexstring(0x87654321);
    //hexstring(earlypc);
    //hexstring(&tA[0]);
    //hexstring(&tB[0]);
    //hexstring(&i);
    //puts("iA: ", 4);
    //hexstring(&step_counter);
    //puts("glitch_counter: ", 16);
    //hexstring(&glitch_counter);
    //for(i=0;i<(sizeof(tA)/sizeof(tA[0]));i++) tA[i] = 0;
    //PUT32(GPSET0, (1<<8));
    //PUT32(GPCLR0, 1<<7);
//    for(i=0;i<(sizeof(tB)/sizeof(tB[0]));i++) tB[i] = 0;
    
/*    PUT32(GPCLR0, 1<<17);
  d  for(counter=0; counter<1000000; counter++) 
        {
        if((counter/100000)%2 == 1)
            PUT32(GPSET0, 1<<17);
        else
            PUT32(GPCLR0, 1<<17);
        }
*/        
   
    //PUT32(IRQ_ENABLE2, 0);
    //PUT32(IRQ_ENABLE1, 0);
    //PUT32(IRQ_ENABLE2, 1<<(49-32));
    //enable_irq();

    
/*    for(ra=0;ra<30000;ra++)
    {
        uart_putc(0x30|(ra&7));
    }

    for(ra=0;ra<100;ra++) uart_putc(0x55);
*/
    //probably a better way to flush the rx fifo.  depending on if and
    //which bootloader you used you might have some stuff show up in the
    //rx fifo.
    while(1)
    {
        if(GET32(UART0_FR)&0x10) break;
        GET32(UART0_DR);
    }

    while(1) {
        //ra=uart_getc();
        if((GET32(UART0_FR)&0x10)==0)
            ra = GET32(UART0_DR);
        else
            ra = 0;
        switch(ra) {
            case '\x0d': uart_putc(0x0A);
                        break;
            case 'p':  
                /*if (GET32(GPEDS0)&(1<<18))
                    {
                    counter++;
                    PUT32(GPEDS0, 1<<18);
                    }
                */            
                uart_putc(0x0A);
                hexstring(step_counter);
                hexstring(glitch_counter);
                break;
            case 'm':
                puts("start\n", 6);
                iA = iB = 0;
                step_counter = 0;
                glitch_counter = 0;
                tB_0 = tA_0 = GET32(TIMER_CLO);
                l0 = l1 = 0;
                while(1) {
	                next_state = 0;	
                    if (GET32(GPEDS0)&(1<<17)) {
                        PUT32(GPEDS0, 1<<17);
                        tA_1 = GET32(TIMER_CLO);
                        if ((tA_1 - tA_0) > 50) {
                            if (l0 == 0) {
                               PUT32(GPFEN0, GET32(GPFEN0)|(1<<17));
                               PUT32(GPREN0, GET32(GPREN0)&(~(1<<17)));
                            }
                            else {
                               PUT32(GPREN0, GET32(GPREN0)|(1<<17));
                               PUT32(GPFEN0, GET32(GPFEN0)&(~(1<<17)));
                            }   
                            l0 = !l0;
                            tA_0 = tA_1;
                            tA[iA++] = tA_1;
                        } else 
                            glitch_counter++;
                    }
/*                    if (GET32(GPEDS0)&(1<<18)) {
                        PUT32(GPEDS0, 1<<18);
                        tB_1 = GET32(TIMER_CLO);
                        if ((tB_1 - tB_0) > 50) {
                            {
                            if (l1 == 0) {
                               PUT32(GPFEN0, GET32(GPFEN0)|(1<<18));
                               PUT32(GPREN0, GET32(GPREN0)&(~(1<<18)));
                            }
                            else {
                               PUT32(GPREN0, GET32(GPREN0)|(1<<18));
                               PUT32(GPFEN0, GET32(GPFEN0)&(~(1<<18)));
                            }   
                            l1 = !l1;
                            next_state |= ((l1<<1) | 8);
                            tB_0 = tB_1;
                            tB[iB++] = tB_1;
                            }
                        }
                        else
                            glitch_counter++;
                    }
*/
                    if (iA == 500 || iB == 500) break;
                    //if (step_counter >= 200) break;
                    
                }
                puts("done\n", 5);
                hexstring(iA);
                hexstring(glitch_counter);
                break;
            // Record the inter edge transition times for 400 transitions
            case 'l':
                i = 1;
                a[0] = GET32(TIMER_CLO);
                while(1) {
                    if (GET32(GPEDS0)&(1<<18)) {
                        PUT32(GPEDS0, 1<<18);
                        a[i]=GET32(TIMER_CLO);
                        if((a[i]-a[i-1])>50)
                            i++;
                    }
                    if(i>=400) break;
                }
                for(i=0;i<400-1;i++) hexstring(a[i+1]-a[i]);
                break;
            case '1': pwm_set(10); puts("duty_cycle=10\n", 14); break;
            case '5': pwm_set(50); puts("duty_cycle=50\n", 14); break;
            case '7': pwm_set(70); puts("duty_cycle=70\n", 14); break;
            case '8': pwm_set(80); puts("duty_cycle=80\n", 14); break;
            case '0': pwm_set(100); puts("duty_cycle=100\n", 14); break;
            case 's': 
                spi_begin();
                i = spi_transfer(i);
                spi_end();
                hexstring(i);
                break;
            case 'i':
                while(GET32(BSC0_S) & BSC_S_RXD) {
                    GET32(BSC0_FIFO);
                };
                hexstring(GET32(BSC0_S));
                puts("Starting i2c test\n\r", 19);
                PUT32(BSC0_S, CLEAR_STATUS); // Reset status bits (see #define)
                PUT32(BSC0_A, 1<<6);
                PUT32(BSC0_DLEN, 1);
                PUT32(BSC0_FIFO, 0xFD);
                
                puts("I2C status: ", 12); 
                hexstring(GET32(BSC0_S));

                PUT32(BSC0_C, START_WRITE);    // Start Write (see #define)
                wait_i2c_done();

                puts("Finished writing\n\r", 18);
                puts("I2C status: ", 12);
                hexstring(GET32(BSC0_S));

                PUT32(BSC0_DLEN, 1);
                PUT32(BSC0_S, CLEAR_STATUS); // Reset status bits (see #define)
                PUT32(BSC0_C, START_READ);    // Start Read after clearing FIFO (see #define)
                wait_i2c_done();

                puts("Read: ", 6);
                hexstring(GET32(BSC0_FIFO));
                puts("Finished i2c test\n\r", 19);
                /*while(1) {
                    PUT32(GPCLR0, 0xffffffff);
                    };*/
                break;
            case '\x00': break;
            default:
                uart_putc(ra);
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
