#ifndef PROG_H_
#define PROG_H_

#define CONF_SIZE 10
#define PGM_STEPS 13

uint8_t ProgGetStep(uint8_t step_num);
int16_t ProgGetParam(uint8_t step_num);
char* ProgGetStepName(uint8_t step_num);
int8_t ConfGet(uint8_t param);
char* ConfGetName(uint8_t param);

//TODO: packed
struct pgm_step {
    uint8_t  op;
    uint16_t param;
};
#define PGM_STEP_SIZE sizeof(pgm_step)
#define PGM_SIZE (PGM_STEP_SIZE * PGM_STEPS)

#define MENU_COUNT (CONF_SIZE + PGM_STEPS)

// program steps values
#define OP_STOP 0 //just wait for button
#define OP_TEMP 1 //keep temperature
#define OP_TIME 2 //count down
#define OP_WAIT 3 //wait with buzzer
#define OP_ERR 0xFF //stop on empty eeprom 


// configuration //
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

#endif
