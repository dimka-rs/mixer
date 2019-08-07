#include <Wire.h>
#include "max6675.h"  //MAX6675 library by Adafruit Version 1.1.0
#include <RotaryEncoder.h> //RotaryEncoder by Matthias Hertel Version 1.1.0
#include <EEPROM.h>
#include "mixer.h"
#include "hardware.h"
#include "display.h"
#include "prog.h"

// debugs //
//#define ERASE_EEPROM
//#define DEBUG
//#define DEBUG_TEMP
//#define TEMP_EMUL
//#define SKIP_VALVE

// constants //

#define DELAY_1S 950
#define DELAY_POLL 200
#define DELAY_VALVE 300 //delay between valve relays switching, reduce EMI
#define TEMP_AVG_SIZE 5
#define TURB_AVG_SIZE 5
#define TURBIDITY_MAX 500
#define BTN_DELAY 200 //ms

// Temperature sensor //
MAX6675 tc(MAX_CK, MAX_CS, MAX_SO);

// Encoder //
RotaryEncoder encoder(ENC_B, ENC_A);

// global vars //
struct state_t st;

// TODO: config only ? //
uint8_t mode = 0;  //0 - scroll, 1 - change
int8_t index = 0; //current element index, can be negative

volatile double a;
volatile double b;
double temp_avg[TEMP_AVG_SIZE];
int turb_avg[TURB_AVG_SIZE];



// functions //
void SetBuzzer(int state)
{
  if(state == 1){
    digitalWrite(BUZZER, LOW);
    st.buzz = 1;
  } else if(state == 0){
    digitalWrite(BUZZER, HIGH);
    st.buzz = 0;
  }
}

void SetMixer(int state)
{
  if(state == 1) {
    digitalWrite(MIXER, LOW);
    st.mix = 1;
  } else if (state == 0){
    digitalWrite(MIXER, HIGH);
    st.mix = 0;  
  }
}

void SetValve(int offset){
  int t = st.valve_tgt;
  t += offset;
  if(t < 0) t = 0;
  if(t > ConfGet(VALVE_TIME_ID)) t = ConfGet(VALVE_TIME_ID);
  st.valve_tgt = t;

  if(st.valve_tgt > st.valve_pos){
    if(st.valve_act == 1) {
      //already moving up, just wait
      delay(DELAY_1S);
      st.valve_pos += 1;
    } else {
      //start moving up, CLOSING valve
      st.valve_act = 1;
      digitalWrite(VALVE_OPEN, HIGH);
      delay(DELAY_VALVE);
      digitalWrite(VALVE_PWR, LOW);
      delay(DELAY_1S - DELAY_VALVE);
      st.valve_pos += 1;
    }
  } else if(st.valve_tgt < st.valve_pos){
    if(st.valve_act == -1){
      //already moving down, just wait
      delay(DELAY_1S);
      st.valve_pos -= 1;
    } else {
      //start moving down, OPENING valve
      st.valve_act = -1;
      digitalWrite(VALVE_OPEN, LOW);
      delay(DELAY_VALVE);
      digitalWrite(VALVE_PWR, LOW);
      delay(DELAY_1S - DELAY_VALVE);
      st.valve_pos -= 1;
    }
  } else {
    //target reached, stop
    st.valve_act = 0;
    digitalWrite(VALVE_PWR, HIGH);
    delay(DELAY_VALVE);
    digitalWrite(VALVE_OPEN, HIGH);
    delay(DELAY_1S - DELAY_VALVE);
  }
}

void ReadTemp()
{
#ifndef TEMP_EMUL
  st.temp_err = '>';
  double t = tc.readCelsius();
  //not sure if it works as expected, bit lib returns NAN if no sensor attached
  if(t == NAN){
    st.temp_err = 'E';
    return;
  }
  //ignore values out of limits
  if(t < ConfGet(TEMP_MIN_ID)){
    st.temp_err = 'E';
    return;
  }
  if(t > ConfGet(TEMP_MAX_ID)){
    st.temp_err = 'E';
    return;
  }
  //shift new value in
  for(int i=1; i < TEMP_AVG_SIZE; i++) {
    temp_avg[i-1] = temp_avg[i];
  }
  temp_avg[TEMP_AVG_SIZE-1] = t;

  //calc average
  t = 0;
  for(int i=0; i < TEMP_AVG_SIZE-1; i++) {
     t += temp_avg[i]/TEMP_AVG_SIZE;
  }

  //check range
  if( t < ConfGet(TEMP_MIN_ID)) {
    st.temp_err = 'E';
  } else
  if(t > ConfGet(TEMP_MAX_ID)) {
    st.temp_err = 'E';
  }

  //calc correction
  double f = a * t + b;
  st.temp = f;
#else //TEMP_EMUL
  switch(millis()%30){ 
    case  0 ... 9: temp = 98.7654321; break;
    case 10 ... 19: temp = 1.123456789; break;
    case 20 ... 29 : temp = 55.55555; break;
  }
#endif //TEMP_EMUL
}

void ReadTurbidity(){
  int t = analogRead(TURBIDITY_PIN);

  //shift new value in
  for(int i=1; i < TURB_AVG_SIZE; i++) {
    turb_avg[i-1] = turb_avg[i];
  }
  turb_avg[TURB_AVG_SIZE-1] = t;

  //calc average
  t = 0;
  for(int i=0; i < TURB_AVG_SIZE-1; i++) {
     t += turb_avg[i]/TURB_AVG_SIZE;
  }
  // convert to percent
  t = t / (TURBIDITY_MAX / 100);
  st.turbidity = t;
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
  st.step = setto;
  Serial.print(">> >> >> >> >> Step: ");
  Serial.println(st.step);  
}

void writeData(){
  extern int8_t conf[];
  extern struct pgm_step pgm[];
  
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
  extern int8_t conf[];
  extern struct pgm_step pgm[];
  
  Serial.println("Loading CONF...");
  
  //overwrite default conf with stored params
  uint8_t t;
  for(uint8_t i = 0; i < CONF_SIZE; i++){
    Serial.print(i);
    Serial.print(": ");
    t = EEPROM.read(i);
    if(t != 0xFF)  conf[i] = t;
    Serial.print(ConfGetName(i));
    Serial.print(": ");
    Serial.println(ConfGet(i));
  }
  // target temperature set to maximum allowed
  st.target = ConfGet(TEMP_MAX_ID);

  Serial.println("Loading PGM...");
  byte op;
  for(int i = 0; i < PGM_STEPS; i++){
    Serial.print(i);
    Serial.print(": ");
    int address = CONF_SIZE + (i * PGM_STEP_SIZE);
    op = EEPROM.read(address);
    if(op != 0xFF){
      Serial.print("_OK_");
      pgm[i].op = op;
      byte paraml = EEPROM.read(address+1);
      byte paramh = EEPROM.read(address+2);
      pgm[i].param = 256 * paramh + paraml;
    }
    Serial.print(ProgGetStepName(ProgGetStep(i)));
    Serial.print(": ");
    Serial.print(ProgGetStep(i));
    Serial.print(": ");
    Serial.print(ProgGetParam(i));
    Serial.println();
  }
  
  //fill temp_avg for a start
  for(int i = 0; i < TEMP_AVG_SIZE; i++){
    temp_avg[i] = ConfGet(TEMP_MAX_ID);
  }
  //fill turb_avg
  for(int i = 0; i < TURB_AVG_SIZE; i++){
    turb_avg[i] = 0; //TODO: choose proper start value
  }
  Serial.println("done");
}

void doConfig(){
  extern int8_t conf[];
  extern struct pgm_step pgm[];
  
  Serial.println("Start config mode");
  DisplayUpdateConf();

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
      DisplayUpdateConf();
    }//mode or pos changed
  }//while(1)
}




/////////////////////////////////////////////////////////////////
//main app
////////////////////////////////////////////////////////////////
void setup() {

    Serial.begin(115200);
    Serial.println("Init...");

    /* LCD indicator */
    Serial.println("LCD");
    DisplayInit();
    
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

    //calc temp correction coeffs
    a = ConfGet(A_INT_ID) + (float)ConfGet(A_FRA_ID) / 100.0;
    b = ConfGet(B_INT_ID) + (float)ConfGet(B_FRA_ID) / 100.0;
    Serial.print("a = ");
    Serial.print(a);
    Serial.print(", b = ");
    Serial.print(b);
    Serial.println();

    #ifdef DEBUG_TEMP
    //lcd.print("TEMP TEST..."); //TODO: add method for arbitrary string
    for(int i = 0; i < 100; i++){
      float f = a * i + b;
      Serial.print(i);
      Serial.print(" => ");
      Serial.print(round(f));
      Serial.println();
    }
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

#ifndef SKIP_VALVE    
    //lcd.print("Open valve..."); //TODO: add display method
    digitalWrite(VALVE_OPEN, LOW);
    delay(DELAY_VALVE);
    digitalWrite(VALVE_PWR, LOW);
    delay(ConfGet(VALVE_TIME_ID)*1000);
    digitalWrite(VALVE_PWR, HIGH);
    delay(DELAY_VALVE);
    digitalWrite(VALVE_OPEN, HIGH);
#endif // SKIP_VALVE

    Serial.println("Main loop");
}

void DoMainLoop(){
  ReadTemp();
  ReadTurbidity();
  DisplayUpdate(&st);
  if(st.cntdown > 0) st.cntdown--;
  
  if(ProgGetStep(st.step) == OP_STOP){
    delay(DELAY_1S);
  } else {
    //control valve once in temp_intvl
    if(st.temp_intvl > ConfGet(TEMP_INTVL_ID)){ 
      st.temp_intvl = 0;
      SetValve((st.temp - st.target) * ConfGet(TEMP_TIME_ID));
    } else {
      st.temp_intvl++;
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
  for(uint8_t s = 0; s < PGM_STEPS; s++){ 
    SetStep(s);
    uint8_t op = ProgGetStep(s);
    uint16_t param = ProgGetParam(s);
    
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
      st.cntdown = param;
      while(st.cntdown) {
        DoMainLoop();
      }
    } // end OP_TIME
    else if(op == OP_TEMP){
      SetMixer(1);
      //keep given temp regardless of time
      st.target = param;
      while(st.temp > st.target) {
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
