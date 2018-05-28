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
    Serial.println("Main loop");
}

void loop() {
  int btn=digitalRead(0);
  if(btn == 1) {
    digitalWrite(MIXER, LOW);  
  } else {
    digitalWrite(MIXER, HIGH);
  }
  Serial.println(btn);
  delay(1000);
  
  
}
