//PWM functionality

//------------------------------------------------------------------------
// Sleep and wait functions
//------------------------------------------------------------------------
/* Sleeps a specified number of miliseconds */
void usleep( unsigned int ms )
{
  unsigned int t0 = GET32(TIMER_CLO);
  //assuming 1Mhz clock
  while (GET32(TIMER_CLO) < t0 + 1000*ms) {}
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
// PWM duty cycle control
//------------------------------------------------------------------------
void pwm_set(unsigned int duty_cycle)
{
  PUT32(PWM_DAT1, duty_cycle);
  usleep(10);
  PUT32(PWM_CTL, GET32(PWM_CTL) | 1);
}
