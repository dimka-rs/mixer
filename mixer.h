#ifndef _MIXER_H_

#define _MIXER_H_

//TODO: packed
struct pgm_step {
    uint8_t  op;
    uint16_t param;
};

#define OP_STOP 0 //just wait for button
#define OP_TEMP 1 //keep temperature
#define OP_TIME 2 //count down
#define OP_WAIT 3 //wait with buzzer
#define OP_ERR 0xFF //stop on empty eeprom 
#define STEPS 13
#define PGM_SIZE (sizeof(pgm_step) * STEPS)

// printable names for program step types
#define PGM_NAMES_SIZE 5
static char* pgm_names[] = {"STOP", "TEMP", "TIME", "WAIT", "ERR!"};

struct pgm_step pgm[STEPS] = {
  {OP_STOP, 0},  //0
  {OP_TIME, 31}, //1
  {OP_TEMP, 40}, //2
  {OP_WAIT, 0},  //3
  {OP_TEMP, 34}, //4
  {OP_TIME, 50}, //5
  {OP_TEMP, 28}, //6
  {OP_TIME, 30}, //7
  {OP_TEMP, 25}, //8
  {OP_TIME, 30}, //9
  {OP_TEMP, 23}, //10
  {OP_TIME, 30}, //11
  {OP_WAIT, 0},  //12
};


#define CONF_SIZE 10
#define CONF_MIN -128
#define CONF_MAX 127
#define TEMP_MIN_ID 0
#define TEMP_MAX_ID 1
#define VALVE_TIME_ID 2
#define VALVE_INV_ID 3
#define TEMP_INTVL_ID 4
#define TEMP_TIME_ID 5
#define A_INT_ID 6
#define A_FRA_ID 7
#define B_INT_ID 8
#define B_FRA_ID 9
int8_t conf[CONF_SIZE] = {5, 95, 20, 0, 10, 1, 1, 0, 0, 0};

//printable names for program params
static char* conf_names[CONF_SIZE] = {"TEMP MIN", "TEMP MAX", "VALVE TIME", "VALVE INV", "TEMP INTVL", "TEMP TIME", "COEF A", "A/100", "COEF B", "B/100"};

#define MENU_COUNT CONF_SIZE + STEPS

#endif
