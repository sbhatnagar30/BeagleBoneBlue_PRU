#include <stdio.h>
#include <signal.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>
#include <stdlib.h>

double duty_cycle;
double r = 0.0;
double v_motor = 0;
double period = 0.001;
double sigma = 0;
double x1_hat = 0;
double x2_hat = 0;
double c_speed = 0;
double alpha = 127.0865;
double beta = 751.8797;
double K11 = 9.975;
double K12 = 0.035;
double K2 = 166.25;
double L11 = 272.91;
double L21 = 5316.4;

int counter = 0;

static int running = 0;
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
  running=0;
  return;
}
int main()
{
  // initialize hardware first
  if(rc_encoder_eqep_init()){
    fprintf(stderr,"ERROR: failed to run rc_encoder_eqep_init\n");
      return -1;
  }
  int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;
  // initialize hardware first
  if(rc_motor_init_freq(freq_hz)) return -1;

  // set signal handler so the loop can exit cleanly
  signal(SIGINT, __signal_handler);
  running=1;
  while(running){
    if (counter % 10 == 0) { //should execute at 1 Khz
      //controller implementation
      c_speed = rc_encoder_eqep_read(1) * (1.0/1920.0) * 1000.0;
      rc_encoder_eqep_write(1,0);
      v_motor = -K11 * x1_hat - K12 * x2_hat - K2 * sigma;
      if (v_motor > 12) {
        v_motor = 12;
      }
      if (v_motor < -12) {
        v_motor = -12;
      }
      x1_hat = x1_hat + .001 * x2_hat - 0.001 * L11 * (x1_hat - c_speed);
      x2_hat = x2_hat - .001 * alpha * x2_hat + .001 * beta * v_motor - .001 * L21 * (x1_hat - c_speed);
      sigma = sigma + .001 * (c_speed - r);
      duty_cycle = (v_motor/12.0);
      //printf("%f\n",duty_cycle);
      rc_motor_set(1,duty_cycle);
    }
    if (counter % 1000 == 0) {
      FILE *fp;
      char buff[8];
      fp = fopen("/home/debian/signal.txt","r");
      fscanf(fp, "%s", buff);
      fclose(fp);
      r = atof(buff);
      printf("%f\n", r);
    }
    counter++;
    rc_usleep(10);
  }
  rc_motor_cleanup();
  rc_encoder_eqep_cleanup();
  return 0;
}


