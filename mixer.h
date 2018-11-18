#ifndef _MIXER_H_
#define _MIXER_H_

//TODO: packed
struct pgm_step {
    uint8_t  op;
    uint16_t param;
};

#define OP_SILENT 0 //just wait for button
#define OP_TEMP 1 //keep temperature
#define OP_TIME 2 //count down
#define OP_WAIT 3 //wait with buzzer
#define OP_STOP 0xFF //stop on empty eeprom 
#define STEPS 13
#define PGM_SIZE (sizeof(pgm_step) * STEPS)

struct pgm_step pgm[STEPS] = {
  {OP_SILENT, 0}, //0
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

// printable names for program step types
#define PGM_NAMES_SIZE 5
static char* pgm_names[] = {"SILENT", "TEMP", "TIME", "WAIT", "ERR"};

#define CONF_SIZE 10
#define TEMP_MIN_ID 0
#define TEMP_MAX_ID 1
#define VALVE_TIME_ID 2
#define VALVE_INV_ID 3
#define TEMP_INTVL_ID 4
uint8_t conf[CONF_SIZE] = {5, 95, 20, 0, 10, 0, 0, 0, 0, 0};

//printable names for program params
static char* conf_names[CONF_SIZE] = {"TEMP MIN", "TEMP MAX", "VALVE TIME", "VALVE INV", "TEMP INTVL", "NONE", "NONE", "NONE", "NONE", "NONE"};

#define MENU_COUNT CONF_SIZE + STEPS

#endif
