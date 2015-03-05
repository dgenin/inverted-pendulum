//I2C functionality

//------------------------------------------------------------------------
/* Waits for I2C to finish transmit/receive. Returns 0 on success or 1
 * on timeout.
 */
void wait_i2c_done() {
  //Wait till done, let's use a timeout just in case
  int timeout = 20;
  while((!(GET32(BSC0_S) & BSC_S_DONE)) && --timeout) {
    usleep(50);
  }
  if(timeout == 0)
    return 1;
  else
    return 0;
}
