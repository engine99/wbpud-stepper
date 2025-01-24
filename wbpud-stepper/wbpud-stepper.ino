#include <Arduino.h>
#include <AccelStepper.h>

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
// #define SER
// #define PHOTO_ANA_PIN A7  // ??
// #define PH_PIN A7         //
// #define LIFT_PIN1 2          // A pin that will be set to high. Lines up with the VCC pin on the 8833
// #define LIFT_PIN2 14         // A pin that will be set to high. Connect with a resistor to pull UP_PIN up.
// #define LIFT_PIN3 15         // A pin that will be set to high. Connect with a resistor to pull DOWN_PIN up.
// #define SINK_PIN 7          // A pin that will be set to low. Lines up with the MD pin on the 8833
// #define UP_PIN 8
// #define DOWN_PIN 9
// #define A1_PIN 3
// #define A2_PIN 4
// #define B1_PIN 5
// #define B2_PIN 6
// #define LED_PIN 13
// #define ANALOG_BITS 10

// For the digispark
// #define PHOTO_ANA_PIN 1 //  Analog pin 1 = PB2 on the digispark
// #define PH_PIN 2    //
// #define A1_PIN 0
// #define A2_PIN 1
// #define B1_PIN 4
// #define B2_PIN 3
// #define ANALOG_BITS 10

// For the Trinket. USBtinyISP programmer.
#define PH_PIN 2
#define PHOTO_ANA_PIN 1 //  Analog pin 1 = PB2 on the trinket
#define A1_PIN 1
#define A2_PIN 0
#define B1_PIN 3
#define B2_PIN 4
#define ANALOG_BITS 10

#define ANALOG_MAX ((1 << ANALOG_BITS) - 1)

// You may need to reduce AVG_HEAD if the motor starts at the wrong time of day
#define AVG_HEAD 0.5  // Weight of the latest sample
#define AVG_TAIL (1.0 - AVG_HEAD)

// Increase DAYLIGHT for day to start earlier / end later. Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT 0.3
#define DAYLIGHT_MARGIN 0.02

// Customize these params according to your motor.
#define STEPS 1024  // for the 28BYJ-48
#define RPM 100

// Customize this param according to your blind.
#define TURNS 1// -8.5  // Turns at sunset

AccelStepper stepper = AccelStepper(AccelStepper::FULL4WIRE, A1_PIN, A2_PIN, B1_PIN, B2_PIN);
bool isOpen = false;  // Make sure your blind is in this position when booting.
float avgLight = isOpen ? 1.0 : 0.0;

// Return the lightness value calculated with exponential moving average.
float sample() {
  float floatRead = ((float)analogRead(PHOTO_ANA_PIN)) / ANALOG_MAX;
  avgLight = (avgLight * AVG_TAIL) + (floatRead * AVG_HEAD);
  return avgLight;
}

void setup() {
#ifdef SER
  Serial.begin(9600);
#endif

  // These may or may not be strictly necessary
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(B1_PIN, OUTPUT);
  pinMode(B2_PIN, OUTPUT);
  pinMode(PH_PIN, INPUT_PULLUP);

#ifdef UP_PIN
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
#endif
#ifdef LIFT_PIN1
  pinMode(LIFT_PIN1, OUTPUT);
  digitalWrite(LIFT_PIN1, 1);
#endif
#ifdef LIFT_PIN2
  pinMode(LIFT_PIN2, OUTPUT);
  digitalWrite(LIFT_PIN2, 1);
#endif
#ifdef LIFT_PIN3
  pinMode(LIFT_PIN3, OUTPUT);
  digitalWrite(LIFT_PIN3, 1);
#endif
#ifdef SINK_PIN1
  pinMode(SINK_PIN1, OUTPUT);
  digitalWrite(SINK_PIN1, 0);
#endif
#ifdef LED_PIN
  digitalWrite(LED_PIN, 1);
#endif

  stepper.setSpeed(RPM);
  stepper.disableOutputs();
}

void move(float position) {
  
#ifdef LED_PIN
  digitalWrite(LED_PIN, 1);
#endif

  stepper.enableOutputs();
  stepper.runToNewPosition(position);
  // Turn off the current because the stepper doesn't need to hold in place.
  stepper.disableOutputs();
#ifdef LED_PIN
  digitalWrite(LED_PIN, 0);
#endif
}

void loop() {

  float s = sample();

#ifdef SER
  Serial.println(s);
#endif

  if (isOpen && s > DAYLIGHT + DAYLIGHT_MARGIN) {  // Sunset happens
#ifdef SER 
    Serial.println("Sunset");
#endif
    move(TURNS * STEPS);
    isOpen = false;
  } else if (!isOpen && s < DAYLIGHT - DAYLIGHT_MARGIN) {  // Sunrise happens
#ifdef SER
    Serial.println("Sunrise");
#endif
    move(0);
    isOpen = true;
  } 

#ifdef UP_PIN
  while (digitalRead(UP_PIN) == 0) {
    stepper.enableOutputs();
    stepper.move(1);
    stepper.disableOutputs();
  }

  while (digitalRead(DOWN_PIN) == 0) {
    stepper.enableOutputs();
    stepper.move(-1);
    stepper.disableOutputs();
  }
#endif

  delay(10);
}