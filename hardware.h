#ifndef HARDWARE_H_
#define HARDWARE_H_

// Relays. LOW is ON //
#define MIXER       7 //mixer motor
#define VALVE_PWR   6 //power cooling valve
#define VALVE_OPEN  5 //open cooling valve
#define BUZZER      4 //buzzer for alarm

// Buttons //
#define BTN_START  15 //A1, green
//BTN_RESET, red
//BTN_GND, black

// TEMP sensor MAX6675 //
#define MAX_CK 13
#define MAX_SO 12
#define MAX_CS 3

// encoder //
#define ENC_A 8
#define ENC_B 9

// Turbidity sensor //
#define TURBIDITY_PIN A7

#endif
