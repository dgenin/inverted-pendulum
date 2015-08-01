extern void PUT32 ( unsigned int, unsigned int );
extern unsigned int GET32 ( unsigned int );
extern void dummy ( unsigned int );

#define PIN_CLR(pin) PUT32(GPCLR0, 1<<pin)
#define PIN_SET(pin) PUT32(GPSET0, 1<<pin)
#define NOW_TIME() GET32(TIMER_CLO)

/* Timer control registers */
#define TIMER_CS 0x20003000
#define TIMER_CLO 0x20003004
#define TIMER_C0 0x2000300C
#define TIMER_C1 0x20003010
#define TIMER_C2 0x20003014
#define TIMER_C3 0x20003018


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
