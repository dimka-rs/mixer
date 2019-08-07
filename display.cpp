#include "Arduino.h"
#include <LiquidCrystal_I2C.h>  //LiquidCrystal I2C by Frank de Brabander Version 1.1.2
#include "display.h"
#include "mixer.h"
#include "prog.h"

// lcd on i2c expander, addr=0x27, size=16x2 //
LiquidCrystal_I2C lcd(0x27, 16, 2);

extern uint8_t mode;
extern int8_t index;

void DisplayInit()
{
    lcd.init();
    lcd.backlight();
    lcd.print("...");
}

void DisplayUpdate(struct state_t *st)
{
  //0123456789ABCDEF
  //T:99.0>70 C:9876
  //S:12 TEMP V11>20

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T:");
  if(st->temp < 10) lcd.print(" ");
  lcd.print(st->temp, 1);
  lcd.print(st->temp_err);
  lcd.print(st->target);
  lcd.setCursor(10,0);
  lcd.print("C:");
  if(st->cntdown < 1000) lcd.print(" ");
  if(st->cntdown < 100) lcd.print(" ");
  if(st->cntdown < 10)  lcd.print(" ");
  lcd.print(st->cntdown);

  lcd.setCursor(0,1);
  lcd.print("S:");
  if(st->step < 10) lcd.print(" ");
  lcd.print(st->step);
  
  lcd.setCursor(5,1);
  // show step type or turbidity analog read
  if(st->show_turbidity == 0) {
    st->show_turbidity = 1;
    lcd.print(ProgGetStepName(st->step));
  } else {
    st->show_turbidity = 0;
    lcd.print(st->turbidity, 1);
    lcd.print('%');
  }

  lcd.setCursor(10,1);
  lcd.print("V");
  if(st->valve_pos < 10) lcd.print(" ");
  lcd.print(st->valve_pos);
  lcd.print(">");
  if(st->valve_tgt < 10) lcd.print(" ");
  lcd.print(st->valve_tgt);
}

void DisplayUpdateConf(){
    lcd.clear();
    lcd.setCursor(0, 0);
    if(index < CONF_SIZE) {
/* 
 //TODO: Serial only available in main
      Serial.print(index);
      Serial.print(": ");
      Serial.print(conf_names[index]);
      Serial.print(": ");
      Serial.println(conf[index]);
*/
      //display conf
      lcd.print(ConfGetName(index));
      lcd.setCursor(11, 0);
      if(index < 10) lcd.print(" ");
      lcd.print(index);
      lcd.print("/");
      lcd.print(CONF_SIZE-1);
      lcd.setCursor(0,1);
      lcd.print(ConfGet(index));
    } else {
      //display steps
/*
      //TODO: serial only available in main
      Serial.print(index);
      Serial.print(": ");
      Serial.print(pgm_names[pgm[index - CONF_SIZE].op]);
      Serial.print(": ");
      Serial.println(pgm[index-CONF_SIZE].param);
*/
      lcd.setCursor(0, 0);
      lcd.print(ProgGetStepName(index - CONF_SIZE));
      lcd.setCursor(11,0);
      if(index-CONF_SIZE < 10) lcd.print(" ");
      lcd.print(index-CONF_SIZE);
      lcd.print("/");
      lcd.print(PGM_STEPS-1);
      lcd.setCursor(0,1);
      lcd.print(ProgGetParam(index-CONF_SIZE));
    }
    
    lcd.setCursor(13, 1);
    if (mode) {
      lcd.print("<->");
    } else {
      lcd.print("   ");
    }
    
}
