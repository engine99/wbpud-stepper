#include <Stepper.h>
/**
* Arduino windowblind puller-upper-downer with a stepper motor. Opens a blind at sunrise, closes at sunset.
*
* I set this up with a Trinket, USBtiny85 programmer, NEMA 17 motor, and SN754410 Quadruple Half-H Driver.
* 
* Blind could be over-rotated and damaged if isOpen or TURNS are not set appropriately.
*
* Copyright 2024 Darren MacDonald
* License GPL3.0. Don't distribute closed-source versions.
*/

// Customize these params according to your board. Make sure your PHOTO_PIN supports analog in and that
// you use the analog pin number, not the board pin number.


// For the Nano
// Trouble Uploading? Close the serial monitor, use (Old bootloader), check your cable is a data cable, Arduino as ISP programmer
#define PHOTO_ANA_PIN A7  // ??
#define PH_PIN A7         //
#define EN_PIN 2          // A pin that will be set to high. Lines up with the VCC pin on the 8833
#define MD_PIN 7          // A pin that will be set to low. Lines up with the MD pin on the 8833
#define UP_PIN 8
#define DOWN_PIN 9
#define A1_PIN 3
#define A2_PIN 4
#define B1_PIN 5
#define B2_PIN 6
#define ANALOG_BITS 10

// For the digispark
// #define PHOTO_ANA_PIN 1 //  Analog pin 1 = PB2 on the trinket
// #define PH_PIN 2    //
// #define A1_PIN 0
// #define A2_PIN 1
// #define B1_PIN 4
// #define B2_PIN 3
// #define ANALOG_BITS 10

// For the Trinket
// #define PHOTO_PIN 1 //  Analog pin 1 = PB2 on the trinket
// #define A1_PIN 1
// #define A2_PIN 0
// #define B1_PIN 3
// #define B2_PIN 4
// #define ANALOG_BITS 10

#define ANALOG_MAX ((1 << ANALOG_BITS) - 1)

// You may need to reduce AVG_HEAD if the motor starts at the wrong time of day
#define AVG_HEAD 0.5  // Weight of the latest sample
#define AVG_TAIL (1.0 - AVG_HEAD)

// Reduce DAYLIGHT for day to start earlier / end later. Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT 0.3
#define DAYLIGHT_MARGIN 0.02

// Customize these params according to your motor.
#define STEPS 2048  // for the 28BYJ-48
#define RPM 10

// Customize this param according to your blind.
#define TURNS -8.5  // Turns at sunset

Stepper stepper = Stepper(STEPS, A1_PIN, A2_PIN, B1_PIN, B2_PIN);
bool isOpen = false;  // Make sure your blind is in this position when booting.
float avgLight = isOpen ? 1.0 : 0.0;

// Return the lightness value calculated with exponential moving average.
float sample() {
  float floatRead = ((float)analogRead(PHOTO_ANA_PIN)) / ANALOG_MAX;
  avgLight = (avgLight * AVG_TAIL) + (floatRead * AVG_HEAD);
  return avgLight;
}

void setup() {
  
  Serial.begin(9600);
  // These may or may not be strictly necessary
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(B1_PIN, OUTPUT);
  pinMode(B2_PIN, OUTPUT);
  pinMode(PH_PIN, INPUT_PULLUP);  // or INPUT_PULLUP if you do not have a pullup resistor in place
  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);

#ifdef EN_PIN
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, 1);
#endif

#ifdef MD_PIN
  pinMode(MD_PIN, OUTPUT);
  digitalWrite(EN_PIN, 1);
#endif

  digitalWrite(13, 1);

  digitalWrite(A1_PIN, 0);
  digitalWrite(A2_PIN, 0);
  digitalWrite(B1_PIN, 0);
  digitalWrite(B2_PIN, 0);

  stepper.setSpeed(RPM);
  delay(2000);
}

void turn(float rotations) {
  
  digitalWrite(13, 1);
  stepper.step(rotations * STEPS);
  // Turn off the current because the stepper doesn't need to hold in place.
  digitalWrite(A1_PIN, 0);
  digitalWrite(A2_PIN, 0);
  digitalWrite(B1_PIN, 0);
  digitalWrite(B2_PIN, 0);
  digitalWrite(13, 0);
}

void loop() {
  float s = sample();
  Serial.println(s);
  digitalWrite(13, isOpen);
  if (isOpen && s < DAYLIGHT - DAYLIGHT_MARGIN) {  // Sunset happens
    Serial.println("Sunset");
    turn(TURNS);                                   // Assuming positive rotations to close
    isOpen = false;
  } else if (!isOpen && s > DAYLIGHT + DAYLIGHT_MARGIN) {  // Sunrise happens
    Serial.println("Sunrise");
    turn(-TURNS);                                          // Assuming negative rotation to open
    isOpen = true;
  } 
  
  while (digitalRead(UP_PIN)) {
    Serial.println("Up");
    turn(1);
  }

  while (digitalRead(DOWN_PIN)) {
    Serial.println("Down");
    turn(-1);
  }

  delay(100);
}