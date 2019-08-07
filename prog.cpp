#include "Arduino.h"
#include "prog.h"


// printable names for program step types
#define PGM_NAMES_SIZE 5
#define PGM_NAME_ERR 4
static char *pgm_names[] = {"STOP", "TEMP", "TIME", "WAIT", "ERR!"};

// program itself: operation and parameter
struct pgm_step pgm[PGM_STEPS] = {
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

// config
int8_t conf[CONF_SIZE] = {5, 95, 20, 0, 10, 1, 1, 0, 0, 0};

uint8_t ProgGetStep(uint8_t step_num)
{
  if(step_num < PGM_STEPS){
    return pgm[step_num].op;
  } else {
    return OP_ERR;
  }
}
int16_t ProgGetParam(uint8_t step_num)
{
  if(step_num < PGM_STEPS){
    return pgm[step_num].param;
  } else {
    return 0;
  }  
}

char* ProgGetStepName(uint8_t step_num)
{
  if(step_num < PGM_STEPS){
    return  pgm_names[ProgGetStep(step_num)];
  } else {
    return pgm_names[PGM_NAME_ERR];
  }
  
}

//printable names for program params
#define CONF_SIZE 10
static char* conf_names[CONF_SIZE] = {"TEMP MIN", "TEMP MAX", "VALVE TIME", "VALVE INV", "TEMP INTVL", "TEMP TIME", "COEF A", "A/100", "COEF B", "B/100"};

int8_t ConfGet(uint8_t param)
{
   return conf[param];
}

char* ConfGetName(uint8_t param)
{
  return conf_names[param];
}
