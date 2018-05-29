#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

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
int step = 0;
int temp = 0;
int mix  = 0;
int cool = 0;
int buzz = 0;

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

int ReadTemp()
{
 return 25;  
}

void UpdateDisplay()
{
  temp = ReadTemp();
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

    lcd.init();                      // Инициализация дисплея  
    lcd.backlight();                 // Подключение подсветки
    UpdateDisplay();
        
    Serial.println("Main loop");
}

void loop() {
  if(ReadBtn(BTN_START) == 1) {
    SetMixer(1); 
    SetCooler(1);
  } else {
    SetMixer(0);
    SetCooler(0);
  }
  UpdateDisplay();
  Serial.println(btn);
  delay(1000);
  
  
}
