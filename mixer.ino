#include <Wire.h> 
#include "max6675.h"
//#include <LEDMatrixDriver.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>
#include "mixer.h"

// constants //
//#define DEBUG

#define VLV_O 10 //time in sec to let valve partially open
#define VLV_C 20 //time in sec to let valve fully close

//indexes
#define VLV_O_ID 10
#define VLV_C_ID 11

//#define DEBUG 1
#define FILTER_TEMP_MIN 0
#define FILTER_TEMP_MAX 100
#define DELAY_1S 950
#define DELAY_POLL 200

/////////////////////////////////////////////
// do not touch text below!
////////////////////////////////////////////

// Choose peripherals //
#define LCD // 1602 LCD over i2c
//#define LMD // MAX7291 8 dig LED indicator
//#define T_DS1820 //use 1-wire DS18B20 sensor
#define T_MAX6675 //use termocouple with MAX6675 adc

// Relays. LOW is ON //
#define MIXER       7 //mixer motor
#define VALVE_PWR   6 //power cooling valve
#define VALVE_OPEN  5 //open cooling valve
#define BUZZER      4 //buzzer for alarm

// Buttons //
#define BTN_START  15 //A1, green
//BTN_RESET, red
//BTN_GND, black

// Temperature sensor 1-wire//
#define ONE_WIRE_BUS 2

// LED indicator //
#ifdef LMD
#define LED_CS 3
#define LED_VCC 14 //A0
#define LED_NUM 1 //number of LED drivers in chain
/* Indicator pins
   GND, black
   VCC, red
   11-MOSI to DIN, green
   13-SCK to CLK, yellow
   3 to CS, blue
   driver = 1
   18 (A4) - VCC enable, active low
*/
LEDMatrixDriver lmd(LED_NUM, LED_CS);
#endif //LMD


/* TEMP sensor MAX6675 */
#ifdef T_MAX6675
#define MAX_CK 13
#define MAX_SO 12
#define MAX_CS 3

MAX6675 tc(MAX_CK, MAX_CS, MAX_SO);
#endif //T_MAX6675

/* DS18B20*/
#ifdef T_DS1820
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
#endif //T_DS1820


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

volatile int step = 0;
volatile int temp = 50;
volatile int mix  = 0;
volatile int cool = 0;
volatile int buzz = 0;
volatile int cntdown = 0;
volatile int mode = 0;  //0 - scroll, 1 - change
volatile int index = 0; //current element index
volatile char temp_err = 'E';


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

void SetCooler(int state)
{
  int t = 0;
  digitalWrite(VALVE_PWR, LOW); //power on
  if(state==1) {
    Serial.print("Open valve, time=");
    digitalWrite(VALVE_OPEN, LOW); //start open
    t = conf[VALVE_TIME_ID];
    Serial.println(t);
    cool=1;
  } else if(state==0) {
    Serial.print("Close valve, time=");
    digitalWrite(VALVE_OPEN, HIGH);
    t = conf[VALVE_TIME_ID];
    Serial.println(t);
    cool=0;
  }
  while(t > 0){
    t--;
    Serial.println(t);
    UpdateDisplay();
    delay(DELAY_1S);
  }
  digitalWrite(VALVE_OPEN, HIGH); //stop open
  digitalWrite(VALVE_PWR, HIGH); //power off
  Serial.println("done");
}

void ReadTemp()
{
  #ifdef T_DS1820
  sensors.requestTemperatures();
  int16_t t = sensors.getTempCByIndex(0); //dev index = 0
  #endif //T_DS1820

  #ifdef T_MAX6675
  int16_t t = tc.readCelsius();
  #endif
  //do some filtering
  temp_err = ' ';
  if( t < FILTER_TEMP_MIN) {
    temp_err = 'E';
    Serial.print("Temperature too low! ");
    Serial.println(t);
    //t = FILTER_TEMP_MIN;
  } else if(t > FILTER_TEMP_MAX) {
    temp_err = 'E';
    Serial.print("Temperature too high! ");
    Serial.println(t);
    //t = FILTER_TEMP_MAX;
  }
  temp = t;
  /*
  // debug - selfheating emulation
  if(cool==1) {
    if(temp > 20) temp -= 1;
  } else {
    if(temp < 110) temp += 1;
  }
  */
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

int DoCountdown()
{
  if(cntdown > 0) {
    cntdown -= 1;
    delay(DELAY_1S);
    return cntdown;
  } else {
    return 0;
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
  ReadTemp();
#ifdef  DEBUG
  Serial.print(millis());
  Serial.print(", Step=");
  Serial.print(step);
  Serial.print(", temp=");
  Serial.print(temp);
  Serial.print(", cnt=");
  Serial.print(cntdown);
  Serial.print(", mix=");
  Serial.print(mix);
  Serial.print(", cool=");
  Serial.print(cool);
  Serial.print(", buzz=");
  Serial.print(buzz);
  Serial.println("");
#endif
  /* swich power off and reinit 
  in case of  accidental reset */
  #ifdef LMD
  digitalWrite(LED_VCC, HIGH);
  delay(50);
  digitalWrite(LED_VCC, LOW);
  lmd.setEnabled(true);
  lmd.setIntensity(7);  // 0 = min, 15 = max
  lmd.setScanLimit(7);  // 0-7: Show 1-8 digits. Beware of currenct restrictions for 1-3 digits! See datasheet.
  lmd.setDecode(0xFF);
  // step //
  if (step >= 10) {
    lmd.setDigit(7, step/10);
  } else {
    lmd.setDigit(7, LEDMatrixDriver::BCD_BLANK);
  }
  lmd.setDigit(6, step%10, true);
  // temp //
  if(temp >= 100) {
    lmd.setDigit(5, temp/100);
  } else {
    lmd.setDigit(5, LEDMatrixDriver::BCD_BLANK);
  }
  if(temp >= 10) {
    lmd.setDigit(4, (temp%100)/10);
  } else {
    lmd.setDigit(4, LEDMatrixDriver::BCD_BLANK);
  }
  lmd.setDigit(3, temp%10, true);
  // cntdown //
  if(cntdown >= 100) {
    lmd.setDigit(2, cntdown/100);
  } else {
    lmd.setDigit(2, LEDMatrixDriver::BCD_BLANK);
  }
  if(cntdown >= 10) {
    lmd.setDigit(1, (cntdown%100)/10);
  } else {
    lmd.setDigit(1, LEDMatrixDriver::BCD_BLANK);
  }
  lmd.setDigit(0, cntdown%10, buzz != 0);
  lmd.display();
  #endif //LMD
  
  #ifdef LCD
  lcd.setCursor(0,0);
  lcd.print("T:");
  if(temp < 100) lcd.print(" ");
  if(temp < 10) lcd.print(" ");
  lcd.print(temp);
  lcd.print(temp_err);

  lcd.setCursor(6,0);
  lcd.print("C:");
  if(cntdown < 100) lcd.print(" ");
  if(cntdown < 10)  lcd.print(" ");
  lcd.print(cntdown);

  lcd.setCursor(12,0);
  lcd.print("S:");
  if(step < 10) lcd.print(" ");
  lcd.print(step);
  
  lcd.setCursor(0,1);
  if(mix==1) {
    lcd.print("MIX");
  } else {
    lcd.print("mix");
  }

  lcd.setCursor(6,1);
  if(cool==1) {
    lcd.print("COOL");
  } else {
    lcd.print("cool");
  }
  
  lcd.setCursor(12,1);
  if(buzz==1) {
    lcd.print("BUZZ");
  } else {
    lcd.print("buzz");
  }
  #endif //LCD
}

void writeData(){
    int address = CONF_SIZE + (index * sizeof(pgm_step));
    EEPROM.write(address, pgm[index].op);
    byte paramh = pgm[index].param / 256;
    byte paraml = pgm[index].param % 256;
    EEPROM.write(address+1, paraml);
    EEPROM.write(address+2, paramh);
}

void loadData(){
  Serial.println("Loading EEPROM...");
  
  //overwrite default conf with stored params
  for(int i = 0; i < CONF_SIZE; i++){
    uint8_t t = EEPROM.read(i);
    if(t != 0xFF)  conf[i] = t;
    #ifdef DEBUG
    Serial.print(conf_names[i]);
    Serial.print(": ");
    Serial.println(conf[i]);
    #endif    
  }

  for(int i = 0; i < STEPS; i++){
    int address = CONF_SIZE + (i * sizeof(pgm_step));
    pgm[i].op = EEPROM.read(address);
    byte paraml = EEPROM.read(address+1);
    byte paramh = EEPROM.read(address+2);
    pgm[i].param = 256 * paramh + paraml;
  }
  //checks & defaults
  #ifdef DEBUG
  for(int i = 0; i < STEPS; i++){
    Serial.print(pgm_names[i]);
    Serial.print(": ");
    Serial.print(pgm[i].op);
    Serial.print(": ");
    Serial.print(pgm[i].param);
    Serial.println();
  }
  #endif
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
            //TODO increase this conf
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
            //TODO decrease this conf
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
    Serial.println("updateDisplayConf");
    lcd.clear();
    lcd.setCursor(0, 0);
    if(index < CONF_SIZE) {
      //display conf
      lcd.print(conf_names[index]);
      if(index < 10) {
        lcd.setCursor(12, 0);
      } else {
        lcd.setCursor(11, 0);
      }
      lcd.print(index);
      lcd.print("/");
      lcd.print(CONF_SIZE);
    } else {
      //display steps
      lcd.setCursor(0, 1);
      lcd.print(pgm_names[pgm[index - CONF_SIZE].op]);
      lcd.setCursor(13, 1);
      if (mode) {
        lcd.print("<->");
      } else {
        lcd.print("   ");
      }
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
    #endif //LCD
    
    /* Inputs */
    Serial.println("inputs");
    pinMode(BTN_START, INPUT_PULLUP);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);

    loadData();
    //check if enter config mode
    if(ReadBtn(BTN_START) == 1) doConfig();
    
    /* Outputs */
    Serial.println("outputs");
    pinMode(MIXER, OUTPUT);
    SetMixer(0);
    pinMode(VALVE_PWR, OUTPUT);
    pinMode(VALVE_OPEN, OUTPUT);
    //SetCooler(0); //will be closed at step 0
    Serial.println("buzzer");
    pinMode(BUZZER, OUTPUT);
    SetBuzzer(0);

    
    /* segment indicator */
    #ifdef LMD
    Serial.println("Segment");
    pinMode(LED_VCC, OUTPUT);
    digitalWrite(LED_VCC, LOW);    
    /* indicator is resetted and inited on every update */
    lmd.setEnabled(true);
    lmd.setIntensity(7);  // 0 = min, 15 = max
    lmd.setScanLimit(7);  // 0-7: Show 1-8 digits. Beware of currenct restrictions for 1-3 digits! See datasheet.
    lmd.setDecode(0xFF);
    #endif //LMD
    
    /* TEMP DS18B20 */
    #ifdef T_DS1820
    Serial.println("DS18B20");
    sensors.begin();
    #endif //T_DS1820

    //UpdateDisplay();
    lcd.print("...");
    Serial.println("Main loop");
}

void loop() {

while(1) {
  //step 0
  //just wait for start button
  SetBuzzer(0);
  delay(300);
  SetMixer(0);
  delay(300);
  SetCooler(0);
  SetStep(0);

  //wait for release
  while(ReadBtn(BTN_START) == 1) {
    UpdateDisplay();
    delay(DELAY_1S);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(DELAY_POLL);
  }
  // Start mixer, it will work all the time
  SetMixer(1);
  
  //loop through program  
  for(uint8_t s = 1; s <= STEPS; s++){ 
    SetStep(s);
    uint8_t op = pgm[s-1].op;
    uint16_t param = pgm[s-1].param;
    
    #ifdef DEBUG
    Serial.print("op: ");
    Serial.print(op);
    Serial.print(", param: ");
    Serial.println(param);
    #endif
    
    if(op == OP_WAIT){
      // turn signal on and wait for button
      SetBuzzer(1);
      //wait for release
      while(ReadBtn(BTN_START) == 1) {
        UpdateDisplay();
        delay(DELAY_1S);
      }
      //wait for press
      while(ReadBtn(BTN_START) == 0) {
        UpdateDisplay();
        delay(DELAY_POLL);
      }
      SetBuzzer(0);
    } //end OP_WAIT
    else if(op == OP_TIME){
      //keep temp for given time
      cntdown = param;
      while(DoCountdown()) {
        UpdateDisplay();
        // no delay here - DoCountdown does it
      }
    } // end OP_TIME
    else if(op == OP_TEMP){
      //keep given temp regardless of time
      while(temp > param) {
        UpdateDisplay(); //refreshes temp
        delay(DELAY_1S); //     
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
