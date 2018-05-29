#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//constants
#define TIME1 10 //time in sec to mix, step 1
#define TEMP2 40 //temp to cool to, step 2

/////////////////////////////////////////////
// do not touch text below!
////////////////////////////////////////////

//Relays. HIGH is ON
#define MIXER 4    //mixer motor
#define COOL_ON  5 //open cooling valve
#define COOL_OFF 6 //close cooling valve
#define BUZZER   7 //buzzer for alarm
//Buttons
#define BTN_START   0 //

//lcd on i2c expander, addr=0x27, size=16x2
LiquidCrystal_I2C lcd(0x27,16,2);

//global vars
volatile int step = 0;
volatile int temp = 50;
volatile int mix  = 0;
volatile int cool = 0;
volatile int buzz = 0;
volatile int cntdown = 0;

//functions
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
    if(temp < 100) temp += 1;
  }
}

void UpdateDisplay()
{
  ReadTemp();
  lcd.setCursor(0,0);
  lcd.print("TEMP:");
  lcd.print(temp);

  lcd.setCursor(9,0);
  lcd.print("STEP:");
  lcd.print(step);
  
  lcd.setCursor(0,1);
  if(mix==1) {
    lcd.print("MIXER");
  } else {
    lcd.print("mixer");
  }

  lcd.setCursor(10,1);
  if(cool==1) {
    lcd.print("COOLER");
  } else {
    lcd.print("cooler");
  }
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
  Serial.print("Step: ");
  Serial.println(step);  
}
/////////////////////////////////////////////////////////////////
//main app
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Init...");
    pinMode(MIXER, OUTPUT);
    pinMode(COOL_ON, OUTPUT);
    pinMode(COOL_OFF, OUTPUT);
    pinMode(BUZZER, OUTPUT);

    pinMode(BTN_START, INPUT_PULLUP);

    lcd.init();
    lcd.backlight();
    UpdateDisplay();
        
    Serial.println("Main loop");
}

void loop() {
  //step 0
  //just wait for start button
  SetStep(0);
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
   while(ReadBtn(BTN_START) == 0) {
    UpdateDisplay();
    delay(1000);
  }
  
}
