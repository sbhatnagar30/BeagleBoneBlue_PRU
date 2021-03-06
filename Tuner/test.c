#include <stdio.h>
#include <signal.h>
#include <rc/encoder_eqep.h>
#include <rc/time.h>
#include <rc/motor.h>

#define pi 3.14159

double duty_cycle;
double v_motor = 0;
double angular_pos = 0;
double v_motor_log[2000];
double angular_pos_log[2000];
int fast = 0;
int medium = 0;
int slow = 0;
int flag = 1;

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
    if (fast % 100 == 0) { //should execute at 1 Khz
      angular_pos = rc_encoder_eqep_read(1) * 2*pi/1920.0;
      if (medium % 200 == 0) {
        //controller implementation
        if (slow % 4 == 0) {
          v_motor = 10;
        }
        if (slow % 4 == 1) {
          v_motor = 0;
        }
        if (slow % 4 == 2) {
          v_motor = -10;
        }
        if (slow % 4 == 3) {
          v_motor = 0;
        }
        duty_cycle = (v_motor/12.0);
        //printf("%f\n",duty_cycle);
        rc_motor_set(1,duty_cycle);
        slow++;
        medium = 0;
      }
      if (slow < 10) {
        v_motor_log[200*slow + medium] = v_motor;
        angular_pos_log[200*slow + medium] = angular_pos;
      }
      if (slow == 10 && flag == 1) {
        printf("writing");
        FILE *fp;
        fp = fopen("data.txt", "w+");
        int i;
        fprintf(fp,"---------------\n");
        for (i = 0; i < 2000; i++) {
          fprintf(fp,"%f\n",v_motor_log[i]);
        }
        fprintf(fp,"---------------\n");
        for (i = 0; i < 2000; i++) {
          fprintf(fp,"%f\n",angular_pos_log[i]);
        }
        fprintf(fp,"---------------\n");
        flag = 0;
        fclose(fp);
        printf("done");
      }
      medium++;
      fast = 0;
    }
    fast++;
    rc_usleep(1);
  }
  rc_motor_cleanup();
  rc_encoder_eqep_cleanup();
  return 0;
}


