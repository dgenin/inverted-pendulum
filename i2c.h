//I2C functionality

//------------------------------------------------------------------------
/* Waits for I2C to finish transmit/receive. Returns 0 on success or 1
 * on timeout.
 */
void i2c_init() {
  while(GET32(BSC0_S) & BSC_S_RXD) {
    GET32(BSC0_FIFO);
  };
  PUT32(BSC0_A, 1<<6);
  PUT32(BSC0_DLEN, 1);
}

int wait_i2c_done() {
  //Wait till done, let's use a timeout just in case
  int timeout = 2000000;
  while((!(GET32(BSC0_S) & BSC_S_DONE)) && --timeout) {
    //usleep(1);
  }
  if(timeout == 0)
    return 1;
  else
    return 0;
}
