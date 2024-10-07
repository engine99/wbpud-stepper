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
#define PHOTO_PIN 1 //  Analog pin 1 = PB2 on the trinket
#define A1_PIN 1
#define A2_PIN 0
#define B1_PIN 3
#define B2_PIN 4
#define ANALOG_BITS 10
#define ANALOG_MAX ((1 << ANALOG_BITS) - 1)

// You may need to reduce AVG_HEAD if the motor starts at the wrong time of day
#define AVG_HEAD 0.5  // Weight of the latest sample 
#define AVG_TAIL (1.0 - AVG_HEAD)

// Reduce DAYLIGHT for day to start earlier / end later. Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT 0.5
#define DAYLIGHT_MARGIN 0.02

// Customize these params according to your motor.
#define STEPS 200
#define RPM 100

// Customize this param according to your blind.
#define TURNS -5     // Turns at sunset

Stepper stepper = Stepper(STEPS, A1_PIN, A2_PIN, B1_PIN, B2_PIN);
bool isOpen = false;  // Make sure your blind is in this position when booting.
float avgLight = isOpen ? 1.0 : 0.0;

// Return the lightness value calculated with exponential moving average.
float sample() {
  float floatRead = ((float) analogRead(PHOTO_PIN)) / ANALOG_MAX;
  avgLight = (avgLight * AVG_TAIL) + (floatRead * AVG_HEAD);
  return avgLight;
}

void setup() {
  // These may or may not be strictly necessary
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(B1_PIN, OUTPUT);
  pinMode(B2_PIN, OUTPUT);
  pinMode(PHOTO_PIN, INPUT);  // or INPUT_PULLUP if you do not have a pullup resistor in place  

  stepper.setSpeed(RPM);
  
  delay(1);
}

void turn(int rotations) {
  stepper.step(rotations * STEPS);

  // Turn off the current because the stepper doesn't need to hold in place.
  digitalWrite(A1_PIN, 0);
  digitalWrite(A2_PIN, 0);
  digitalWrite(B1_PIN, 0);
  digitalWrite(B2_PIN, 0);
}

void loop() {
  float s = sample();
  if (isOpen && s < DAYLIGHT - DAYLIGHT_MARGIN) {   // Sunset happens
    turn(TURNS);                            // Assuming positive rotations to close 
    isOpen = false;
  } else if (!isOpen && s > DAYLIGHT + DAYLIGHT_MARGIN) {  // Sunrise happens
    turn(-TURNS);                           // Assuming negative rotation to open
    isOpen = true;
  }
  delay(1);
}