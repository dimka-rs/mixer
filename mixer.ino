#include <Wire.h> 
#include "max6675.h"
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>
#include "mixer.h"

// debugs //
//#define ERASE_EEPROM
//#define DEBUG
//#define DEBUG_TEMP

// constants //
#define LCD
#define T_MAX6675
#define DELAY_1S 950
#define DELAY_POLL 200
#define DELAY_VALVE 300 //delay between valve relays switching, reduce EMI
#define TEMP_AVG_SIZE 5

// Relays. LOW is ON //
#define MIXER       7 //mixer motor
#define VALVE_PWR   6 //power cooling valve
#define VALVE_OPEN  5 //open cooling valve
#define BUZZER      4 //buzzer for alarm

// Buttons //
#define BTN_START  15 //A1, green
//BTN_RESET, red
//BTN_GND, black

/* TEMP sensor MAX6675 */
#ifdef T_MAX6675
#define MAX_CK 13
#define MAX_SO 12
#define MAX_CS 3

MAX6675 tc(MAX_CK, MAX_CS, MAX_SO);
#endif //T_MAX6675

// lcd on i2c expander, addr=0x27, size=16x2 //
#ifdef LCD
LiquidCrystal_I2C lcd(0x27,16,2);
#endif //LCD

// encoder //
#define ENC_A 8
#define ENC_B 9
#define BTN_DELAY 200 //ms
RotaryEncoder encoder(ENC_B, ENC_A);


// global vars //
volatile int target = 0;
volatile int step = 0;
volatile int temp = 50;
volatile int mix  = 0;
volatile int vpwr = 0;
volatile int vdir = 0;
volatile int buzz = 0;
volatile int cntdown = 0;
volatile int valve_pos = 0;
volatile int valve_tgt = 0;
volatile int valve_act = 0;
volatile int mode = 0;  //0 - scroll, 1 - change
volatile int index = 0; //current element index
volatile int temp_intvl = 0;
volatile char temp_err = 'E';
int temp_avg[TEMP_AVG_SIZE];

//functions
void SetBuzzer(int state)
{
  if(state==1){
    digitalWrite(BUZZER, LOW);
    buzz=1;
  } else if(state==0){
    digitalWrite(BUZZER, HIGH);
    buzz=0;
  }
}

void SetMixer(int state)
{
  if(state==1) {
    digitalWrite(MIXER, LOW);
    mix = 1;
  } else if (state==0){
    digitalWrite(MIXER, HIGH);
    mix = 0;  
  }
}

void SetValve(int offset){
  int t = valve_tgt;
  t += offset;
  if(t < 0) t = 0;
  if(t > conf[VALVE_TIME_ID]) t = conf[VALVE_TIME_ID];
  valve_tgt = t;

  if(valve_tgt > valve_pos){
    if(valve_act == 1) {
      //already moving up, just wait
      delay(DELAY_1S);
      valve_pos += 1;
      return 1;
    } else {
      //start moving up, CLOSING valve
      valve_act = 1;
      digitalWrite(VALVE_OPEN, HIGH);
      delay(DELAY_VALVE);
      digitalWrite(VALVE_PWR, LOW);
      delay(DELAY_1S - DELAY_VALVE);
      valve_pos += 1;
      return 1;
    }
  } else if(valve_tgt < valve_pos){
    if(valve_act == -1){
      //already moving down, just wait
      delay(DELAY_1S);
      valve_pos -= 1;
      return 1;
    } else {
      //start moving down, OPENING valve
      valve_act = -1;
      digitalWrite(VALVE_OPEN, LOW);
      delay(DELAY_VALVE);
      digitalWrite(VALVE_PWR, LOW);
      delay(DELAY_1S - DELAY_VALVE);
      valve_pos -= 1;
    }
  } else {
    //target reached, stop
    valve_act = 0;
    digitalWrite(VALVE_PWR, HIGH);
    delay(DELAY_VALVE);
    digitalWrite(VALVE_OPEN, HIGH);
    delay(DELAY_1S - DELAY_VALVE);
  }
}

/*
int DriveValve(){
  int offset = 0;
  if(offset == 0) return 0;
  
  if(conf[VALVE_INV_ID]) offset *= -1;
  #ifdef DEBUG
  Serial.print("Offset: ");
  Serial.println(offset);
  #endif
  digitalWrite(VALVE_PWR, LOW); //power on
  vpwr = 1;
  if(offset > 0) {
    offset = 0;
    //decrease cooling, OPEN BYPASS
    Serial.println("Open valve");
    digitalWrite(VALVE_OPEN, LOW); //start open
    vdir = 0;
  } else if(offset < 0) {
    offset = 0;
    //increase cooling, CLOSE BYPASS
    Serial.println("Close valve");
    digitalWrite(VALVE_OPEN, HIGH);
    vdir = 1;    
  }
  UpdateDisplay();
  delay(DELAY_1S);
  digitalWrite(VALVE_OPEN, HIGH); //stop open
  digitalWrite(VALVE_PWR, HIGH); //power off
  vdir = 0;
  vpwr = 0;
  return 1;
}
*/

void ReadTemp()
{
  temp_err = ' ';
  int16_t t;
  #ifdef T_MAX6675
  t = tc.readCelsius();
  #endif
  //ignore values out of limits
  if(t < conf[TEMP_MIN_ID]){
    temp_err = 'E';
    return;
  }
  if(t > conf[TEMP_MAX_ID]){
    temp_err = 'E';
    return;
  }
  //shift new value in
  for(int i=1; i < TEMP_AVG_SIZE; i++) {
    temp_avg[i-1] = temp_avg[i];
  }
  temp_avg[TEMP_AVG_SIZE-1] = t;

  //calc average
  t = 0;
  int rem = 0;
  for(int i=0; i < TEMP_AVG_SIZE-1; i++) {
     t += temp_avg[i]/TEMP_AVG_SIZE;
     rem += temp_avg[i]%TEMP_AVG_SIZE;
  }
  t += rem / TEMP_AVG_SIZE;

  //check range
  if( t < conf[TEMP_MIN_ID]) {
    temp_err = 'E';
  } else
  if(t > conf[TEMP_MAX_ID]) {
    temp_err = 'E';
  }

  //calc correction
  float a = conf[A_INT_ID] + (float)conf[A_FRA_ID] / 100.0;
  float b = conf[B_INT_ID] + (float)conf[B_FRA_ID] / 100.0;
  float f = a * t + b;
  temp = round(f);
}

int ReadBtn(int btn)
{
  int status = digitalRead(btn);
  if(status==1) {
    //pulled up = not pressed
    return 0;
  } else {
    //pulled down = pressed
    return 1;
  }
}


void SetStep(int setto)
{
  step = setto;
  Serial.print(">> >> >> >> >> Step: ");
  Serial.println(step);  
}

void UpdateDisplay()
{
  #ifdef  DEBUG1
  Serial.print(millis());
  Serial.print(", Step=");
  Serial.print(step);
  Serial.print(", temp=");
  Serial.print(temp);
  Serial.print(", cnt=");
  Serial.print(cntdown);
  Serial.print(", mix=");
  Serial.print(mix);
  Serial.print(", vpwr=");
  Serial.print(vpwr);
  Serial.print(", vdir=");
  Serial.print(vdir);
  Serial.print(", buzz=");
  Serial.print(buzz);
  Serial.println("");
  #endif
  //0123456789ABCDEF
  //T:99E>>70 C:9876
  //S:12 TEMP V11>20
  #ifdef LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("T:");
  if(temp < 10) lcd.print(" ");
  lcd.print(temp);
  lcd.print(temp_err);
  lcd.print(">>");
  lcd.print(target);
  lcd.setCursor(10,0);
  lcd.print("C:");
  if(cntdown < 1000) lcd.print(" ");
  if(cntdown < 100) lcd.print(" ");
  if(cntdown < 10)  lcd.print(" ");
  lcd.print(cntdown);

  lcd.setCursor(0,1);
  lcd.print("S:");
  if(step < 10) lcd.print(" ");
  lcd.print(step);
  
  lcd.setCursor(5,1);
  lcd.print(pgm_names[pgm[step].op]);

  /*
  if(mix==1) {
    lcd.print("M");
  } else {
    lcd.print("m");
  }
  if(vpwr == 1) {
    lcd.print("V");
  } else {
    lcd.print("v");
  }
  if(vdir == 1) {
    lcd.print("D");
  } else {
    lcd.print("d");
  }
  if(buzz==1) {
    lcd.print("B");
  } else {
    lcd.print("b");
  }*/

  lcd.setCursor(10,1);
  lcd.print("V");
  if(valve_pos < 10) lcd.print(" ");
  lcd.print(valve_pos);
  lcd.print(">");
  if(valve_tgt < 10) lcd.print(" ");
  lcd.print(valve_tgt);
  #endif //LCD
}

void writeData(){
  if(index < CONF_SIZE){
    #ifdef DEBUG
    Serial.print("Write conf[");
    Serial.print(index);
    Serial.print("]=");
    Serial.print(conf[index]);
    Serial.println();
    #endif
    EEPROM.write(index, conf[index]);
  } else {
    int address = CONF_SIZE + ((index-CONF_SIZE) * sizeof(pgm_step));
    #ifdef DEBUG
    Serial.print("Write pgm[");
    Serial.print(index-CONF_SIZE);
    Serial.print("]=");
    Serial.print(pgm[index-CONF_SIZE].op);
    Serial.print(",");
    #endif
    EEPROM.write(address, pgm[index-CONF_SIZE].op);
    delay(100);
    byte paramh = pgm[index-CONF_SIZE].param / 256;
    byte paraml = pgm[index-CONF_SIZE].param % 256;
    #ifdef DEBUG
    Serial.print(paramh);
    Serial.print(",");
    Serial.print(paraml);
    Serial.println();
    #endif
    EEPROM.write(address+1, paraml);
    delay(100);
    EEPROM.write(address+2, paramh);
    delay(100);
  }
}

void loadData(){
  Serial.println("Loading CONF...");
  
  //overwrite default conf with stored params
  uint8_t t;
  for(int i = 0; i < CONF_SIZE; i++){
    Serial.print(i);
    Serial.print(": ");
    t = EEPROM.read(i);
    if(t != 0xFF)  conf[i] = t;
    Serial.print(conf_names[i]);
    Serial.print(": ");
    Serial.println(conf[i]);
  }
  // target temperature set to maximum allowed
  target = conf[TEMP_MAX_ID];

  Serial.println("Loading PGM...");
  byte op;
  for(int i = 0; i < STEPS; i++){
    Serial.print(i);
    Serial.print(": ");
    int address = CONF_SIZE + (i * sizeof(pgm_step));
    op = EEPROM.read(address);
    if(op != 0xFF){
      Serial.print("_OK_");
      pgm[i].op = op;
      byte paraml = EEPROM.read(address+1);
      byte paramh = EEPROM.read(address+2);
      pgm[i].param = 256 * paramh + paraml;
    }
    Serial.print(pgm_names[pgm[i].op]);
    Serial.print(": ");
    Serial.print(pgm[i].op);
    Serial.print(": ");
    Serial.print(pgm[i].param);
    Serial.println();
  }
  
  //fill temp_avg for a start
  for(int i = 0; i < TEMP_AVG_SIZE; i++){
    temp_avg[i] = conf[TEMP_MAX_ID];
  }
  Serial.println("done");
}

void doConfig(){
  Serial.println("Start config mode");
  updateDisplayConf();

  static int btnReleased = 0;
  static int modeChanged = 0;
  static int pos = 0;
  static int newPos = 0;
  while(1) {
    if(digitalRead(BTN_START) == 1){
      btnReleased = 1;
    }

    if(btnReleased && (digitalRead(BTN_START) == 0)){
      int now = millis();
      while (millis() - now < BTN_DELAY);
      if(digitalRead(BTN_START) == 0){
        //btn still pressed
        btnReleased = 0;
        modeChanged = 1;
        if (mode) {
          mode = 0;
        } else {
          mode = 1;
        }
      }
    }

    encoder.tick();
    newPos = encoder.getPosition();
    if ((pos != newPos) || (modeChanged)) {
      if(modeChanged && !mode){
        //save data on exit
        writeData();
      }
      modeChanged = 0;
      if (newPos > pos) {
        if(mode){
          // edit mode, increase value
          if(index < CONF_SIZE){
            if(conf[index] < CONF_MAX) conf[index]++;
          } else {
            pgm[index - CONF_SIZE].param++;
          }
        } else {
          //scroll mode, increase index
          index++;
          if (index >= MENU_COUNT) index = 0;
        }
      } else if (newPos < pos){
        if (mode){
          //edit mode, decrease params
          if(index < CONF_SIZE){
            if(conf[index] > CONF_MIN) conf[index]--;
          } else {
            pgm[index - CONF_SIZE].param--;
          }
        } else {
          //scroll mode, decrease index
          index--;
          if (index < 0) index = MENU_COUNT - 1;
        }
      }
      pos = newPos;
      updateDisplayConf();
    }//mode or pos changed
  }//while(1)
}

void updateDisplayConf(){
    lcd.clear();
    lcd.setCursor(0, 0);
    if(index < CONF_SIZE) {
      Serial.print(index);
      Serial.print(": ");
      Serial.print(conf_names[index]);
      Serial.print(": ");
      Serial.println(conf[index]);
      //display conf
      lcd.print(conf_names[index]);
      lcd.setCursor(11, 0);
      if(index < 10) lcd.print(" ");
      lcd.print(index);
      lcd.print("/");
      lcd.print(CONF_SIZE-1);
      lcd.setCursor(0,1);
      lcd.print(conf[index]);
    } else {
      //display steps
      Serial.print(index);
      Serial.print(": ");
      Serial.print(pgm_names[pgm[index - CONF_SIZE].op]);
      Serial.print(": ");
      Serial.println(pgm[index-CONF_SIZE].param);
      lcd.setCursor(0, 0);
      lcd.print(pgm_names[pgm[index - CONF_SIZE].op]);
      lcd.setCursor(11,0);
      if(index-CONF_SIZE < 10) lcd.print(" ");
      lcd.print(index-CONF_SIZE);
      lcd.print("/");
      lcd.print(STEPS-1);
      lcd.setCursor(0,1);
      lcd.print(pgm[index-CONF_SIZE].param);
    }
    
    lcd.setCursor(13, 1);
    if (mode) {
      lcd.print("<->");
    } else {
      lcd.print("   ");
    }
    
}


/////////////////////////////////////////////////////////////////
//main app
////////////////////////////////////////////////////////////////
void setup() {

    Serial.begin(115200);
    Serial.println("Init...");

    /* LCD indicator */
    #ifdef LCD
    Serial.println("LCD");
    lcd.init();
    lcd.backlight();
    lcd.print("...");
    #endif //LCD
    
    /* Inputs */
    Serial.println("inputs");
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    #ifdef ERASE_EEPROM
    Serial.print("Erasing EEPROM... ");
    for (int i = 0 ; i < EEPROM.length() ; i++) {
      EEPROM.write(i, 0xFF);
    }
    Serial.println("done");
    #endif

    loadData();
    //check if enter config mode
    if(ReadBtn(BTN_START) == 1) doConfig();

    #ifdef DEBUG_TEMP
    lcd.print("TEMP TEST...");
    float a = conf[A_INT_ID] + (float)conf[A_FRA_ID] / 100.0;
    float b = conf[B_INT_ID] + (float)conf[B_FRA_ID] / 100.0;
    Serial.print("a = ");
    Serial.print(a);
    Serial.print(", b = ");
    Serial.print(b);
    Serial.println();
    for(int i = 0; i < 100; i++){
      float f = a * i + b;
      Serial.print(i);
      Serial.print(" => ");
      Serial.print(round(f));
      Serial.println();
    }
    while(1);
    #endif
    
    /* Outputs */
    Serial.println("outputs");
    pinMode(MIXER, OUTPUT);
    SetMixer(0);
    pinMode(VALVE_PWR, OUTPUT);
    digitalWrite(VALVE_PWR, HIGH);
    pinMode(VALVE_OPEN, OUTPUT);
    digitalWrite(VALVE_OPEN, HIGH);
    
    Serial.println("buzzer");
    pinMode(BUZZER, OUTPUT);
    SetBuzzer(0);
    
    lcd.print("Open valve...");
    digitalWrite(VALVE_OPEN, LOW);
    delay(DELAY_VALVE);
    digitalWrite(VALVE_PWR, LOW);
    delay(conf[VALVE_TIME_ID]*1000);
    digitalWrite(VALVE_PWR, HIGH);
    delay(DELAY_VALVE);
    digitalWrite(VALVE_OPEN, HIGH);
    
    Serial.println("Main loop");
}

void DoMainLoop(){
  ReadTemp();
  UpdateDisplay();
  if(cntdown > 0) cntdown--;
  
  if(pgm[step].op == OP_STOP){
    delay(DELAY_1S);
  } else {
    //control valve once in temp_intvl
    if(temp_intvl > conf[TEMP_INTVL_ID]){ 
      temp_intvl = 0;
      SetValve((temp - target) * conf[TEMP_TIME_ID]);
    } else {
      temp_intvl++;
      SetValve(0);
    }
  }
}


void loop() {

while(1) {
  SetBuzzer(0);
  delay(300);
  SetMixer(0);
  delay(300);
  
  //loop through program  
  for(uint8_t s = 0; s < STEPS; s++){ 
    SetStep(s);
    uint8_t op = pgm[s].op;
    uint16_t param = pgm[s].param;
    
    #ifdef DEBUG
    Serial.print("op: ");
    Serial.print(pgm_names[op]);
    Serial.print(", param: ");
    Serial.println(param);
    #endif

    if(op == OP_STOP){
      SetMixer(0);
      SetBuzzer(0);
      //wait for release
      while(ReadBtn(BTN_START) == 1) {
        DoMainLoop();
      }
      //wait for press
      while(ReadBtn(BTN_START) == 0) {
        DoMainLoop();
      }
    } //OP_STOP
    else if(op == OP_WAIT){
      SetMixer(1);
      // turn signal on and wait for button
      SetBuzzer(1);
      //wait for release
      while(ReadBtn(BTN_START) == 1) {
        DoMainLoop();
      }
      //wait for press
      while(ReadBtn(BTN_START) == 0) {
        DoMainLoop();
      }
      SetBuzzer(0);
    } //end OP_WAIT
    else if(op == OP_TIME){
      SetMixer(1);
      //keep temp for given time
      cntdown = param;
      while(cntdown) {
        DoMainLoop();
      }
    } // end OP_TIME
    else if(op == OP_TEMP){
      SetMixer(1);
      //keep given temp regardless of time
      target = param;
      while(temp > target) {
        DoMainLoop();     
      }
    } // end OP_TEMP
    else {
      // wrong op code, exit loop
      Serial.print("Wrong opcode: ");
      Serial.println(op);
      break;
    }

  //end of  pgm for 
  }
  
//end of while(1)
}

//end of void loop()
}
