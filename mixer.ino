#include <Wire.h> 
#include "max6675.h"
//#include <LEDMatrixDriver.hpp>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal_I2C.h>

//constants
#define TIME1 31 //time in sec to mix, step 1
#define TEMP2 40 //temp to cool to, step 2
#define TEMP4 34 //temp to cool to, step 4
#define TIME5 50 //time in sec to mix, step 5
#define TEMP6 28 //temp to cool to, step 6

//#define DEBUG 1
#define FILTER_TEMP_MIN (0)
#define FILTER_TEMP_MAX (100)

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
#define VALVE_ON    6 //power cooling valve
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

//global vars
volatile int step = 0;
volatile int temp = 50;
volatile int mix  = 0;
volatile int cool = 0;
volatile int buzz = 0;
volatile int cntdown = 0;

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
  //always turn power on
  digitalWrite(VALVE_ON, LOW);
  if(state==1) {
    digitalWrite(VALVE_OPEN, LOW);
    cool=1;
  } else if(state==0) {
    digitalWrite(VALVE_OPEN, HIGH);
    cool=0;
  }
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
  if( t >= FILTER_TEMP_MIN and t <= FILTER_TEMP_MAX)
  {
    temp = t;
  } else {
    Serial.print("Wrong temperature: ");
    Serial.println(t);
  }
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
    delay(950);
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

/////////////////////////////////////////////////////////////////
//main app
////////////////////////////////////////////////////////////////
void setup() {

    Serial.begin(115200);
    Serial.println("Init...");

    /* Outputs */
    pinMode(MIXER, OUTPUT);
    SetMixer(0);
    pinMode(VALVE_ON, OUTPUT);
    pinMode(VALVE_OPEN, OUTPUT);
    SetCooler(0);
    pinMode(BUZZER, OUTPUT);
    SetBuzzer(0);

    /* Inputs */
    pinMode(BTN_START, INPUT_PULLUP);

    /* segment indicator */
    #ifdef LMD
    pinMode(LED_VCC, OUTPUT);
    digitalWrite(LED_VCC, LOW);    
    /* indicator is resetted and inited on every update */
    lmd.setEnabled(true);
    lmd.setIntensity(7);  // 0 = min, 15 = max
    lmd.setScanLimit(7);  // 0-7: Show 1-8 digits. Beware of currenct restrictions for 1-3 digits! See datasheet.
    lmd.setDecode(0xFF);
    #endif //LMD
    
    /* LCD indicator */
    #ifdef LCD
    lcd.init();
    lcd.backlight();
    #endif //LCD

    /* TEMP DS18B20 */
    #ifdef T_DS1820
    sensors.begin();
    #endif //T_DS1820

    UpdateDisplay();

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
    delay(950);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(200);
  }

  // ----- step 1 -----
  //mix for TIME1
  SetStep(1);
  SetMixer(1);
  cntdown = TIME1;
  while(DoCountdown()) {
    UpdateDisplay();
    // no delay here - DoCountdown does it
  }

  // ----- step 2 -----
  //start cooling until TEMP2
  SetStep(2);
  SetCooler(1);
  while(temp > TEMP2) {
    UpdateDisplay(); //refreshes temp
    delay(950);
  }

  // ----- step 3 -----
  //wait for button
  SetStep(3);
  SetBuzzer(1);
  //wait for release
  while(ReadBtn(BTN_START) == 1) {
    UpdateDisplay();
    delay(950);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(200);
  }
  SetBuzzer(0);

  // ----- step 4 -----
  //cool until TEMP4
  SetStep(4);
  SetCooler(1);
  while(temp > TEMP4) {
    UpdateDisplay(); //refreshes temp
    delay(950);
  }

  // ----- step 5 -----
  //mix for TIME5
  SetStep(5);
  SetMixer(1);
  cntdown = TIME5;
  while(DoCountdown()) {
    UpdateDisplay();
    // no delay here - DoCountdown does it
  }

  // ----- step 6 -----
  //cooldown 1 deg/sec down to TEMP6
  //TODO control cooldown speed
  SetStep(6);
  SetCooler(1);
  while(temp > TEMP6) {
    UpdateDisplay(); //refreshes temp
    delay(950);
  }

  // ----- step 7 -----
  //buzz until button pressed
  SetStep(7);
  SetBuzzer(1);
  //wait for release
  while(ReadBtn(BTN_START) == 1) {
    UpdateDisplay();
    delay(950);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(200);
  }
  //return to step 0
}
}
