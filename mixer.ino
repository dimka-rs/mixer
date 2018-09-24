#include <Wire.h> 

#include "max6675.h"
#include <LEDMatrixDriver.hpp>

//#include <LiquidCrystal_I2C.h>

//constants
#define TIME1 31 //time in sec to mix, step 1
#define TEMP2 40 //temp to cool to, step 2
#define TEMP4 34 //temp to cool to, step 4
#define TIME5 100 //time in sec to mix, step 5
#define TEMP6 28 //temp to cool to, step 6

#define DEBUG 1

/////////////////////////////////////////////
// do not touch text below!
////////////////////////////////////////////

//Relays. HIGH is ON
#define MIXER     4    //mixer motor
#define COOL_ON   5 //open cooling valve
#define COOL_OFF  6 //close cooling valve
#define BUZZER    7 //buzzer for alarm
//Buttons
#define BTN_START  19 //A5
#define BTN_STOP   2

/* Indicator pins
   11-MOSI to DIN,
   13-SCK to CLK,
   3 to CS,
   driver = 1
*/
LEDMatrixDriver lmd(1, 3);


/*
TEMP sensor:
D14 - CK
D8 - SO
D9 - CS
*/

//MAX6675 tc(thermoCLK, thermoCS, thermoDO);
MAX6675 tc(14, 9, 8);


//lcd on i2c expander, addr=0x27, size=16x2
//LiquidCrystal_I2C lcd(0x27,16,2);

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
    digitalWrite(BUZZER, HIGH);
    buzz=1;
  } else if(state==0){
    digitalWrite(BUZZER, LOW);
    buzz=0;
  }
}

void SetMixer(int state)
{
  if(state==1) {
    digitalWrite(MIXER, HIGH);
    mix = 1;
  } else if (state==0){
    digitalWrite(MIXER, LOW);
    mix = 0;  
  }
}

void SetCooler(int state)
{
  
  if(state==1) {
    digitalWrite(COOL_OFF,LOW);
    delay(1000);
    digitalWrite(COOL_ON,HIGH);
    cool=1;
  } else if(state==0) {
    digitalWrite(COOL_ON,LOW);
    delay(1000);
    digitalWrite(COOL_OFF,HIGH);
    cool=0;
  }
  
}

void ReadTemp()
{
  if(cool==1) {
    if(temp > 20) temp -= 1;
  } else {
    if(temp < 110) temp += 1;
  }
}

void UpdateDisplay()
{
  ReadTemp();
#ifdef  DEBUG
  Serial.print("Step=");
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
  /* step */
  if (step >= 10) {
    lmd.setDigit(7, step/10);
  } else {
    lmd.setDigit(7, LEDMatrixDriver::BCD_BLANK);
  }
  lmd.setDigit(6, step%10, true);
  /* temp */
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
  /* cntdown */
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
  
/*  lcd.setCursor(0,0);
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
  }*/
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
  if(cntdown>0) {
    cntdown -= 1;
    delay(1000);
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
/////////////////////////////////////////////////////////////////
//main app
////////////////////////////////////////////////////////////////
void setup() {

    Serial.begin(115200);
    Serial.println("Init...");

    /* Outputs */
    pinMode(MIXER, OUTPUT);
    pinMode(COOL_ON, OUTPUT);
    pinMode(COOL_OFF, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    /* Inputs */
    pinMode(BTN_START, INPUT_PULLUP);
    
    /* segment indicator */
    lmd.setEnabled(true);
    lmd.setIntensity(7);  // 0 = min, 15 = max
    lmd.setScanLimit(7);  // 0-7: Show 1-8 digits. Beware of currenct restrictions for 1-3 digits! See datasheet.
    lmd.setDecode(0xFF);
  
    /* LCD indicator */
    //lcd.init();
    //lcd.backlight();

    UpdateDisplay();

    Serial.println("Main loop");
}

void loop() {
  
  //step 0
  //just wait for start button
  SetStep(0);
  SetBuzzer(0);
  SetMixer(0);
  SetCooler(0);

  //wait for release
  while(ReadBtn(BTN_START) == 1) {
    UpdateDisplay();
    delay(1000);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(1000);
  }

  //step 1
  //mix for TIME1
  SetStep(1);
  SetMixer(1);
  cntdown = TIME1;
  while(DoCountdown()) {
    UpdateDisplay();
    // no delay here - DoCountdown does it
  }

  //step 2
  //start cooling until TEMP2
  SetStep(2);
  SetCooler(1);
  while(temp > TEMP2) {
    UpdateDisplay(); //refreshes temp
    delay(1000);
  }

  //step 3
  //wait for button
  SetStep(3);
  SetBuzzer(1);
  //wait for release
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(1000);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 1) {
    UpdateDisplay();
    delay(1000);
  }
  SetBuzzer(0);
  //DEBUG
  temp=80;

  //step 4
  //cool until TEMP4
  SetStep(4);
  SetCooler(1);
  while(temp > TEMP4) {
    UpdateDisplay(); //refreshes temp
    delay(1000);
  }

  //step 5
  //mix for TIME5
  SetStep(5);
  SetMixer(1);
  cntdown = TIME5;
  while(DoCountdown()) {
    UpdateDisplay();
    // no delay here - DoCountdown does it
  }

  //step 6
  //cooldown 1 deg/sec down to TEMP6
  //TODO control cooldown speed
  SetStep(6);
  SetCooler(1);
  while(temp > TEMP6) {
    UpdateDisplay(); //refreshes temp
    delay(1000);
  }

  //step 7
  //buzz until button pressed
  SetStep(7);
  SetBuzzer(1);
  //wait for release
  while(ReadBtn(BTN_START) == 1) {
    UpdateDisplay();
    delay(1000);
  }
  //wait for press
  while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(1000);
  }
  SetBuzzer(0);
}
