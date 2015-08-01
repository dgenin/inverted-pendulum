#include "dev_mem_map.h"
#define ABROAD extern
#include "driver.h"
#undef ABROAD

//------------------------------------------------------------------------
/* Pins 7 and 8 should only be manipulated in motor_run and motor_stop 
   Sets direction of the motor
*/
void motor_run(int direction)
{
  if (control_dir == direction)
    return;
  if (1 == direction)
    {
      control_dir = 1;
      PIN_CLR(7);
      PIN_SET(8);
    }
  else if (-1 == direction)
    {
      control_dir = -1;
      PIN_CLR(8);
      PIN_SET(7);
    }
}
//------------------------------------------------------------------------
void motor_stop()
{
  PIN_SET(7);
  PIN_SET(8);
  control_dir = 0;
}
void motor_stop1()
{
  PIN_CLR(7);
  PIN_CLR(8);
  control_dir = 0;
}
//------------------------------------------------------------------------
void move_to_limit(int direction)
{
  unsigned start_head;

  start_head = t_head;
  set_motor_voltage(75);
  motor_run(direction);
  
  while(t_head == start_head)
    ;
  while(((NOW_TIME() - t[t_head]) < STALL_TIMEOUT) || ((NOW_TIME() - t[t_head]) < STALL_TIMEOUT))
    ;
  motor_stop();
  hexstring(x);
  //hexstring(t[t_head]);
  //hexstring(NOW_TIME());
}
//------------------------------------------------------------------------
void move(int direction, int ticks)
{
  int i,j;

  //puts("move\n\r");
  i = x + direction*ticks;
  j = 0;
  motor_run(direction);
  while((direction*(x-i))<0) {
    //TODO: Figure out why the stall check fails closed like a valve
    /*if(!(((NOW_TIME() - t[t_head]) < STALL_TIMEOUT) || ((NOW_TIME() - t[t_head]) < STALL_TIMEOUT)))
      {
	puts("stalled\n\r");
	break;
      }
    */
    j++;
    usleep(1);
    //if (j>ticks) break;
  }
  motor_stop();
  //puts("done\n\r");
  //hexstring(i);
  //hexstring(x);
}
//------------------------------------------------------------------------
void move_to(int position)
{
  if (x>position)
    move(-1, x-position);
  else
    move(1, position-x);
}
