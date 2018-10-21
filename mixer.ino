#include <Wire.h> 
#include "max6675.h"
//#include <LEDMatrixDriver.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

// constants //
// on adding new param:
// + increment DATA_SIZE
// + define new param
// + define new id
// + update names arran
// + update checks and defauls section
//#define DEBUG
#define DATA_SIZE 7 //total params
#define TIME1 31 //time in sec to mix, step 1
#define TEMP2 40 //temp to cool to, step 2
#define TEMP4 34 //temp to cool to, step 4
#define TIME5 50 //time in sec to mix, step 5
#define TEMP6 28 //temp to cool to, step 6
#define VLV_O 10 //time in sec to let valve partially open
#define VLV_C 20 //time in sec to let valve fully close

//indexes
#define TIME1_ID 0
#define TEMP2_ID 1
#define TEMP4_ID 2
#define TIME5_ID 3
#define TEMP6_ID 4
#define VLV_O_ID 5
#define VLV_C_ID 6

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
int16_t data[DATA_SIZE];
char* names[] = {"TIME1:", "TEMP2:", "TEMP4:", "TIME5:", "TEMP6:", "VLV-O:", "VLV-C:"};

volatile int step = 0;
volatile int temp = 50;
volatile int mix  = 0;
volatile int cool = 0;
volatile int buzz = 0;
volatile int cntdown = 0;
volatile int mode = 0;  //0 - scroll, 1 - change
volatile int index = 0; //current element index


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
    t = data[VLV_O_ID];
    Serial.println(t);
    cool=1;
  } else if(state==0) {
    Serial.print("Close valve, time=");
    digitalWrite(VALVE_OPEN, HIGH);
    t = data[VLV_C_ID];
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
  if( t < FILTER_TEMP_MIN) {
    Serial.print("Temperature too low! ");
    Serial.println(t);
    t = FILTER_TEMP_MIN;
  } else if(t > FILTER_TEMP_MAX) {
    Serial.print("Temperature too high! ");
    Serial.println(t);
    t = FILTER_TEMP_MAX;
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
  if(temp < 100) lcd.print("0");
  if(temp < 10) lcd.print("0");
  lcd.print(temp);

  lcd.setCursor(6,0);
  lcd.print("C:");
  if(cntdown < 100) lcd.print("0");
  if(cntdown < 10)  lcd.print("0");
   
  lcd.print(cntdown);

  lcd.setCursor(12,0);
  lcd.print("S:");
  if(step < 10) lcd.print("0");
  lcd.print(step);
  
  lcd.setCursor(0,1);
  if(mix==1) {
    lcd.print("MIX");
  } else {
    lcd.print("mix");
  }

  lcd.setCursor(6,1);
  if(buzz==1) {
    lcd.print("BUZZ");
  } else {
    lcd.print("buzz");
  }

  lcd.setCursor(12,1);
  if(cool==1) {
    lcd.print("COOL");
  } else {
    lcd.print("cool");
  }
  #endif //LCD
}

void writeData(){
    int address = index * 2;
    byte datah = data[index] / 256;
    byte datal = data[index] % 256;
    EEPROM.write(address, datal);
    EEPROM.write(address+1, datah);
}

void loadData(){
  Serial.println("Loading EEPROM...");
  int address;
  for(int i = 0; i < DATA_SIZE; i++){
    address = i * 2;
    byte datal = EEPROM.read(address);
    byte datah = EEPROM.read(address+1);
    data[i] = 256 * datah + datal;
  }
  //checks & defaults
  if(data[TIME1_ID] <= 0) data[TIME1_ID] = TIME1;
  if(data[TEMP2_ID] <= 0) data[TEMP2_ID] = TEMP2;
  if(data[TEMP4_ID] <= 0) data[TEMP4_ID] = TEMP4;
  if(data[TIME5_ID] <= 0) data[TIME5_ID] = TIME5;
  if(data[TEMP6_ID] <= 0) data[TEMP6_ID] = TEMP6;
  if(data[VLV_O_ID] <= 0) data[VLV_O_ID] = VLV_O;
  if(data[VLV_C_ID] <= 0) data[VLV_C_ID] = VLV_C;
  for(int i=0; i < DATA_SIZE; i++){
    Serial.print(names[i]);
    Serial.print(" = ");
    Serial.println(data[i]);
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
           data[index]++;
        } else {
          index++;
          if (index >= DATA_SIZE) index = 0;
        }
      } else if (newPos < pos){
        if (mode){
          data[index]--;
        } else {
          index--;
          if (index < 0) index = DATA_SIZE - 1;
        }
      }
      pos = newPos;
      updateDisplayConf();
    }//mode or pos changed
  }//while(1)
}

void updateDisplayConf(){
    Serial.println("updateDisplayConf");
    lcd.setCursor(0, 0);
    lcd.print(names[index]);
    lcd.setCursor(15, 0);
    lcd.print(index);
    lcd.print("  ");
    lcd.setCursor(0, 1);
    lcd.print(data[index]);
    lcd.print("    ");
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

  // ----- step 1 -----
  //mix for TIME1
  SetStep(1);
  SetMixer(1);
  cntdown = data[TIME1_ID];
  while(DoCountdown()) {
    UpdateDisplay();
    // no delay here - DoCountdown does it
  }

  // ----- step 2 -----
  //start cooling until TEMP2
  SetStep(2);
  SetCooler(1);
  while(temp > data[TEMP2_ID]) {
    UpdateDisplay(); //refreshes temp
    delay(DELAY_1S);
  }

  // ----- step 3 -----
  //wait for button
  SetStep(3);
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

  // ----- step 4 -----
  //cool until TEMP4
  SetStep(4);
  //SetCooler(1); already open
  while(temp > data[TEMP4_ID]) {
    UpdateDisplay(); //refreshes temp
    delay(DELAY_1S);
  }

  // ----- step 5 -----
  //mix for TIME5
  SetStep(5);
  SetMixer(1);
  cntdown = data[TIME5_ID];
  while(DoCountdown()) {
    UpdateDisplay();
    // no delay here - DoCountdown does it
  }

  // ----- step 6 -----
  //cooldown 1 deg/sec down to TEMP6
  
  SetStep(6);
  //SetCooler(1); already open
  int16_t target;
  UpdateDisplay(); //refreshes temp
  while(temp > data[TEMP6_ID]) {
    target = temp - 1;
    //control cooldown speed
    /* disabled. done via partial open
    for(int i = 0 ; i < 59 ; i++){
      //this loop will try to hold temp stable for 1 minute
      UpdateDisplay(); //refreshes temp
      if (temp <= target){
        SetCooler(0);
      } else {
        SetCooler(1); 
      }
      delay(DELAY_1S);
    }
    */
  }

  // ----- step 7 -----
  //buzz until button pressed
  SetStep(7);
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
  //return to step 0
}
}
