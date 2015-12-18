//UART functionality

// UART documentation
//GPIO14  TXD0 and TXD1
//GPIO15  RXD0 and RXD1
//alt function 5 for uart1
//alt function 0 for uart0

//(3000000 / (16 * 115200) = 1.627
//(0.627*64)+0.5 = 40
//int 1 frac 40

//------------------------------------------------------------------------
/* Returns the length of a string. If the string is longer that 255,
 * 255 is returned.
 */
unsigned int strlen(const char *str){
  unsigned int i;
  for(i=0; i<255 && str[i]!='\x0'; i++)
    ;
  return i;
}
//------------------------------------------------------------------------
// UART output functions
//------------------------------------------------------------------------
/* Waits for the UART to be ready to send and then sends the character. 
 */
void uart_putc(unsigned int c){
  while(1)
    {
      if((GET32(UART0_FR)&TX_FIFO_FULL)==0) break;
    }
  PUT32(UART0_DR,c);
}
//------------------------------------------------------------------------
/* Prints the string to the serial port. The string is limited to 255 
 * characters.
 */
void puts(const char *str){
  unsigned int i, n = strlen(str);
  for(i = 0; i < n; uart_putc(str[i++]));
}
//------------------------------------------------------------------------
/* Prints the hexadecimal representation of d to serial port. */
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
/* Prints the hexadecimal representation of d followed by \n\r. */
void hexstring(unsigned int d){
  hexstrings(d);
  uart_putc(0x0D);
  uart_putc(0x0A);
}
void hexstring_signed(unsigned int d){
  if(d < (0xFFFFFFFF >> 1)) hexstring(d);
  else {uart_putc('-'); hexstring(0xFFFFFFFF - d);}
}
//------------------------------------------------------------------------
// UART input functions
//------------------------------------------------------------------------
/* Waits for UART to become readable and reads a single character. */
unsigned int uart_getc ( void )
{
  while(1)
    {
      if((GET32(UART0_FR)&RX_FIFO_EMPTY)==0) break;
    }
  return(GET32(UART0_DR));
}
//------------------------------------------------------------------------
/* Checks if the UART is readable and reads a single character.
 * If UART is not readable returns -1.
 */
int uart_getc_nb ( void )
{
   if((GET32(UART0_FR)&RX_FIFO_EMPTY)==0)
     return(GET32(UART0_DR));
   else
     return -1;
}
//------------------------------------------------------------------------
/* Reads digits from the serial port until the next digit would make
 * the total greater than 2^32-1 or a non-numerical digit is encountered.
 */
unsigned int uart_read_int( void ) {
  unsigned int ret = 0;
  unsigned int next_digit = 0;

  ret = 0;
  while(10*ret < (1<<32)-1)
    {
      next_digit = uart_getc();
      if ((next_digit - 0x30) >= 0 && (next_digit - 0x30) <= 9)
	ret = 10*ret + (next_digit-0x30);
      else
	return ret;
    }
  return ret;
}
//------------------------------------------------------------------------
void uart_flush_rx_fifo ( void )
{
  while(1)
    {
      if(GET32(UART0_FR)&0x10) break;
      GET32(UART0_DR);
    }
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
