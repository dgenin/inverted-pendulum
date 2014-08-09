
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

//GPIO14  TXD0 and TXD1
//GPIO15  RXD0 and RXD1
//alt function 5 for uart1
//alt function 0 for uart0

//(3000000 / (16 * 115200) = 1.627
//(0.627*64)+0.5 = 40
//int 1 frac 40

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
    PUT32(GPFSEL0,ra);

    //disable pull up/down to gpio 14, 15
    PUT32(GPPUD,0);
    for(ra=0;ra<150;ra++) dummy(ra);
    PUT32(GPPUDCLK0,(1<<14)|(1<<15));
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
    PUT32(PWM_DAT1, 20);
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
    pwm_init();
    uart_init();
    hexstring(0x87654321);
    hexstring(earlypc);
    hexstring(&tA[0]);
    hexstring(&tB[0]);
    for(i=0;i<(sizeof(tA)/sizeof(tA[0]));i++) tA[i] = 0;
    for(i=0;i<(sizeof(tB)/sizeof(tB[0]));i++) tB[i] = 0;
    
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
            // Count glitches until 200 edge transitions are counted
            case 'm':
                iA = iB = 0;
                step_counter = 0;
                glitch_counter = 0;
                tB_0 = tA_0 = GET32(TIMER_CLO);
                l0 = l1 = 0;
                current_state = 0b00;//((GET32(GPLEV0)&(1<<17))|(GET32(GPLEV0)&(1<<18)))>>17;
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
                            next_state |= (l0 | 8);
                            tA_0 = tA_1;
                            tA[iA++] = tA_1;
                        } else 
                            glitch_counter++;
                    }
                    if (GET32(GPEDS0)&(1<<18)) {
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
/*
                    if ((next_state&8)==0) continue;
                    next_state &= 3;

                    //hexstring(current_state);
                    //hexstring(next_state);
                    //hexstring(step_counter);
                    if ((current_state^next_state)!=0b11) {
                        switch(current_state) {
                            case 0b00:
                                if (next_state == 0b01)
                                    step_counter++;
                                else
                                    step_counter--;
                                break;
                            case 0b01:
                                if (next_state == 0b11)
                                    step_counter++;
                                else
                                    step_counter--;
                                break;
                            case 0b11:
                                if (next_state == 0b10)
                                    step_counter++;
                                else
                                    step_counter--;
                                break;
                            case 0b10:
                                if (next_state == 0b00)
                                    step_counter++;
                                else
                                    step_counter--;
                                break;
                            };
                       current_state = next_state;
                    }
                    else
                        puts("error\r\n", 7);

                    //puts("\n",1);
*/
                    if (iA == 5000 || iB == 5000) break;
                    //if (step_counter >= 200) break;
                    
                }
                puts("done\n", 5);
                hexstring(step_counter);
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
