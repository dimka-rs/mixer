#ifndef MIXER_H_
#define MIXER_H_

struct state_t {
  int step = 0;
  char show_turbidity = 0;
  double turbidity = 0;
  double temp = 50;
  int target = 0;
  int temp_intvl = 0;
  char temp_err = '>'; //displays > or E
  int mix  = 0;
  int vpwr = 0;
  int vdir = 0;
  int buzz = 0;
  int cntdown = 0;
  int valve_pos = 0;
  int valve_tgt = 0;
  int valve_act = 0;
};

#endif
