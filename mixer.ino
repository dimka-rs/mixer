#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);

//Relays. HIGH is ON
#define MIXER 4    //mixer motor
#define COOL_ON  5 //open cooling valve
#define COOL_OFF 6 //close cooling valve
#define SMTH_EL  7 //4th relay not used
//Buttons
#define BUTTON   0 //
 
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Init...");
    pinMode(MIXER, OUTPUT);
    pinMode(COOL_ON, OUTPUT);
    pinMode(COOL_OFF, OUTPUT);
    //pinMode(SMTH_EL, OUTPUT);

    pinMode(BUTTON, INPUT_PULLUP);

    lcd.init();                      // Инициализация дисплея  
    lcd.backlight();                 // Подключение подсветки
    lcd.setCursor(0,0);              // Установка курсора в начало первой строки
    lcd.print("TEMP:xx  STEP:xx");       // Набор текста на первой строке
    lcd.setCursor(0,1);              // Установка курсора в начало второй строки
    lcd.print("mixer     cooler");  
    
    Serial.println("Main loop");
}

void SetMixer(int state)
{
  if(state==1) {
    digitalWrite(MIXER, LOW);
    lcd.setCursor(0,1);
    lcd.print("mixer");
  } else if (state==0){
    digitalWrite(MIXER, HIGH);
    lcd.setCursor(0,1);
    lcd.print("MIXER");  
  }
}

void loop() {
  int btn=digitalRead(0);
  if(btn == 1) {
    SetMixer(1);  
  } else {
    SetMixer(0);
  }
  Serial.println(btn);
  delay(1000);
  
  
}
