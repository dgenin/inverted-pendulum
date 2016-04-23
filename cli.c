//#include<stdio.h>
#include "cli.h"
#define ABROAD
#include "driver.h"
#undef ABROAD

char cli_buffer[256];
int buffer_pos;

void cli_init() {
  buffer_pos = 0;
  puts("\r\n> ");
}

static void cli_process() {
  char str[1024];

  if(strncasecmp(cli_buffer, "home", 4) == 0) {puts("homing\n"); homing();}
  else if(strncasecmp(cli_buffer, "bal", 3) == 0) balance();
  else 
    if(strncasecmp(cli_buffer, "lean ", 5) == 0) {
    int i;
    sscanf(cli_buffer + 5, "%d", &i);
    lean = i;
    snprintf(str, sizeof(str), "lean = %d\n", lean);
    puts(str);
    } else
      if(strncasecmp(cli_buffer, "a2s ", 4) == 0) {
	  int i;
	  sscanf(cli_buffer + 4, "%d", &i);
	  a2_scale = i;
	  snprintf(str, sizeof(str), "a2_scale = %d\n", a2_scale);
	  puts(str);
	}
	else 
	  if(strncasecmp(cli_buffer, "cb ", 3) == 0) {
	      int i;
	      sscanf(cli_buffer + 3, "%d", &i);
	      center_bias = i;
	      snprintf(str, sizeof(str), "center_bias = %d\n", center_bias);
	      puts(str);
	  }
	  else 
	    if(strncasecmp(cli_buffer, "a2o ", 4) == 0) {
	      int i;
	      sscanf(cli_buffer + 4, "%d", &i);
	      a2_offset = i;
	      snprintf(str, sizeof(str), "a2_offset = %d\n", a2_offset);
	      puts(str);
	    } else
	      if(strncasecmp(cli_buffer, "ah ", 3) == 0) {
		int i;
		sscanf(cli_buffer + 3, "%d", &i);
		ah = i;
		snprintf(str, sizeof(str), "ah = %d\n", ah);
		puts(str);
	    }
	      else
		if(strncasecmp(cli_buffer, "conf", 4) == 0) {
		  snprintf(str, sizeof(str), "lean = %d\n\ra2s = %d\n\ra2o = %d\n\rcb = %d\n\rah = %d\n\r", lean, a2_scale, a2_offset, center_bias, ah);
		  puts(str);
		}
		else
		  if(strncasecmp(cli_buffer, "ang", 3) == 0) {
		    int i = 0;
		    for(i=0; i<1000; i++) {
		      snprintf(str, sizeof(str), "%u\r\n", read_angle());
		      puts(str);
		      usleep(10);
		    }
		  }
		  else
		    if(strncasecmp(cli_buffer, "res", 3) == 0) {
		      encoder_head = 0;
		    }
		    else
		      if(strncasecmp(cli_buffer, "enc", 3) == 0) {
			int i = 0;
			for(i=0; i<C_BUFFER_SIZE; i++) {
			  snprintf(str, sizeof(str), "%d %d\r\n", encoder[i]&1, encoder[i]&2);
			  puts(str);
			}
		      }
		      else
			if(strncasecmp(cli_buffer, "tck", 3) == 0) {
			  snprintf(str, sizeof(str), "%d x= %d\r\n", tick_error_count, x);
			  puts(str);
			}
		      else
			puts("home -- home the carriage\r\n"
			     "bal -- balance\r\n"
			     "lean <int> -- added to CENTER constant to bias carriage to the center of the track\r\n"
			     "a2s <int> -- used to scale the square of the angle in the motor voltage computation\r\n"
			     "a2o <int> -- used to offset the square of the angle in the motor voltage computation\r\n"
			     "cb <int> -- center bias\r\n"
			     "ah <int> -- angle hysteresis\r\n"
			     "ang -- print current angel\r\n"
			     "conf -- print current values of config variables\r\n"
			     "res -- reset head pointer of the encoder tick buffer, i.e., restart recording\r\n"
			     "enc -- dump contents of the encoder tick buffer\r\n"
			     "tck -- display tick error count\r\n"
			     );
}

void cli(int c) {
  switch(c) {
  case 127:
  case '\b': 
    if(buffer_pos>0) {
      buffer_pos--;
      puts("\b \b");
    }
    break;

  case '\r':
  case '\n':
    cli_buffer[buffer_pos] = 0;
    puts("\r\n");
    cli_process();
    puts("\r\n> ");
    buffer_pos = 0;
    break;
  
  default:
    uart_putc(c);
    if(buffer_pos<sizeof(cli_buffer)) {
      cli_buffer[buffer_pos] = c;
      buffer_pos++;
    }
  }
}
