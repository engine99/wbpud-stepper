#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <EEncoder.h>
#include <StateMachine.h>

/**
* Arduino windowblind puller-upper-downer with a stepper motor. Opens a blind at sunrise, closes at sunset.
*
* Blind could be over-rotated and damaged if isOpen or TURNS are not set appropriately.
*
* Copyright 2024 Darren MacDonald
* License GPL3.0. Don't distribute closed-source versions.
*/

// Customize these params according to your board. Make sure your PHOTO_PIN supports analog in and that
// you use the analog pin number, not the board pin number.

// Define either A1_PIN, A2_PIN, B1_PIN, and B2_PIN, or DIR_PIN, STEP_PIN, and EN_PIN depending on whether you have a motor driver or stepper driver.

// For the Nano
// Trouble Uploading? Close the serial monitor, use (Old bootloader), check your cable is a data cable, Arduino as ISP programmer
// #define BAUD 9600
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
// #define LED1_PIN 13
// #define ANALOG_BITS 10

// For the digispark. USBasp programmer
// #define PHOTO_ANA_PIN 1 //  Analog pin 1 = PB2 on the digispark
// #define PH_PIN 2    //
// #define A1_PIN 0
// #define A2_PIN 1
// #define B1_PIN 4
// #define B2_PIN 3
// #define ANALOG_BITS 10

// For the Trinket. USBtinyISP programmer.
// With an 8835 programmer
// #define PH_PIN 2
// #define PHOTO_ANA_PIN 1 //  Analog pin 1 = PB2 on the trinket
// #define A1_PIN 1
// #define A2_PIN 0
// #define B1_PIN 3
// #define B2_PIN 4
// #define ANALOG_BITS 10
// With a A988 driver
// #define PH_PIN 3
// #define PHOTO_ANA_PIN 3 //  Analog pin 3 = PB3 on the trinket
// #define EN_PIN 0
// #define STEP_PIN 1
// #define LED1_PIN 1
// #define DIR_PIN 2
// #define ANALOG_BITS 10

// For the Pico. Use JLink programmer.
// #define BAUD 115200
// // With DRV8825
// #define PH_PIN 27
// #define PHOTO_ANA_PIN 27 //  Analog pin 2 = PB28 on the pico
// #define UP_PIN 1
// #define DOWN_PIN 2
// #define EN_PIN 9
// #define DIR_PIN 3
// #define STEP_PIN 4
// //#define LIFT_PIN1 5
// #define M0_PIN 8
// #define M1_PIN 7
// #define M2_PIN 6
// #define MICROSTEPS_PER_STEP 4 // 2^X steps per step e.g. 4 = 16 microsteps per step
// #define LED1_PIN 25
// #define EEPROM_ADDRESS 0
// #define ANALOG_BITS 10
// With DRV8835
// #define PH_PIN 28
// #define PHOTO_ANA_PIN 2 //  Analog pin 2 = PB28 on the pico
// #define A1_PIN 8
// #define A2_PIN 27
// #define B1_PIN 26
// #define B2_PIN 9
// #define LIFT_PIN1 6
// #define LIFT_PIN2 22


// For the Pico Tiny RP2040
// Board  -> 'Waveshare RP2040 Zero'

// #define LED1_PIN 20
// #define LED2_PIN 19
// #define LED_ON 0
// #define LED_OFF 1
// #define ANALOG_BITS 10
// #define EEPROM_ADDRESS 0
// #define UP_PIN A1
// #define DOWN_PIN A2
// With L293D
// #define EN_PIN 6
// #define H1_PIN 4
// #define H2_PIN 5
// With A988
// #define SIXTEENTH_STEP_IS_111 1
// #define EN_PIN 7
// #define M1_PIN 6
// #define M2_PIN 5
// #define M3_PIN 4
// #define LIFT_PIN2 3
// #define LIFT_PIN3 2
// #define STEP_PIN 1
// #define DIR_PIN 0


// For the RP2040-zero
// Board 'Raspberry Pi Pico/RP2040/RP2035' -> 'Waveshare RP2040 Zero'
// To enter bootloading mode, hold reset, hold boot, release reset, release boot
#define BAUD 115200
#define PH_PIN 29         //A2 is pin 28 on the Tiny2040
#define PHOTO_ANA_PIN 29  // On some (Pi RP2040?) this should be the PB pin i.e. PH_PIN, otherwise it's the A pin number
#define NEOPIXEL_PIN 16
#define UP_PIN 27
#define DOWN_PIN 28
#define EEPROM_ADDRESS 0
#define ANALOG_BITS 12  // Tiny2040 is 12, RP2040 is 10
// // With the L293D and an encoder
// #define EN_PIN 6
// #define H1_PIN 4
// #define H2_PIN 5
// #define SINK_PIN1 14
// #define ENC1_PIN 15
// #define ENC2_PIN 26
// #define LIFT_PIN1 27
// // With the A988
#define SIXTEENTH_STEP_IS_111 1
#define EN_PIN 7
#define M1_PIN 6
#define M2_PIN 5
#define M3_PIN 4
#define LIFT_PIN2 3
#define LIFT_PIN3 2
#define STEP_PIN 1
#define DIR_PIN 0

#define ANALOG_MAX ((1 << ANALOG_BITS) - 1)

// You may need to reduce AVG_HEAD if the motor starts at the wrong time of day
#define AVG_HEAD 0.2  // Weight of the latest sample
#define AVG_TAIL (1.0 - AVG_HEAD)

// Increase DAYLIGHT for day to start earlier / end later. Light increases conductivity in the photocell, which is connected to ground, so more light lowers the
// voltage on the pin which is pulled up by a resistor
// Upper limit (least light) is NOT 1.0.
// On a trinket (5V):
//      Blk   Dark  Light
// 10K  3.2   3.17  1.0
// 75K  3.15  3.07  .99
//  1M  3.14  3.05  .84
#define INITIAL_SUNSET_LIGHT 0.5  // The analog reading value 0-1. This is the voltage divided by the logic voltage (3.3 or 5V).

// Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT_MARGIN 0.02

// Customize these params according to your motor.
//#define STEPS_PER_ROTATION 64 * 2 * 64  // for the 28BYJ-48
// #define STEPS_PER_ROTATION 200UL// * 4 // for the JK28HS32-0674
#define STEPS_PER_ROTATION 200UL  // for the 42STH34-0354A
#define MICROSTEP_MODE 4          // 2^X steps per step e.g. 3 = 8 microsteps per step // Check below for pin setting fixes
#define RPM 60
#define BTN_FACTOR 1.2  // RPM is multiplied by this wnen button pressed
// #define ENC_REDUCTION 157 // Gear reduction of the motor wrt the encoder x callbacks per encoder revolution

// Customize this param according to your blind. (4cm per turn)
#define TURNS -24  // Turns at sunset

#define SLOW_INTERVAL 1000  // milliseconds between output, light sense ...

#define MICROSTEPS_PER_STEP (1 << MICROSTEP_MODE)

bool isOpen = true;                   // Make sure your blind is in this position when booting.;
float avgLight = isOpen ? 0.0 : 0.8;  // isOpen means more light, low value
float sunsetLight = INITIAL_SUNSET_LIGHT;
long lastReadTime = 0;  // Count the number of reads since last output. Don't want to output every time.
StateMachine stateMachine;
State *moving, *resting;
float maxSpeed = (STEPS_PER_ROTATION * RPM * MICROSTEPS_PER_STEP / 60.f);

#ifdef A1_PIN
// For drivers like the A4988
AccelStepper stepper(AccelStepper::FULL4WIRE, A1_PIN, A2_PIN, B1_PIN, B2_PIN);
#endif

#ifdef STEP_PIN
// For drivers like the A4988, DRV8825, STSPIN220
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
#endif

#ifdef ENC1_PIN
EEncoder encoder(ENC1_PIN, ENC2_PIN);
long position = 0;
#endif

#ifdef NEOPIXEL_PIN
CRGB led;
#endif

void setup() {

  Serial.begin(BAUD);
  while (!Serial);
  int x = Serial.getWriteError();
  Serial.flush();
  Serial.println(x);

  #ifdef LED1_PIN
    pinMode(LED1_PIN, OUTPUT);
    digitalWrite(LED1_PIN, LED_ON);  // blue to indicate booting or operating
  #endif
  #ifdef LED2_PIN
    pinMode(LED2_PIN, OUTPUT);
    digitalWrite(LED2_PIN, LED_ON);  // LED2 to indicate power
  #endif

  #ifdef NEOPIXEL_PIN
    FastLED.addLeds<WS2812, NEOPIXEL_PIN>(&led, 1);
    led = CRGB::Red;
    FastLED.show();
  #endif

  stateMachine = StateMachine();
  moving = stateMachine.addState(&runMoving);
  resting = stateMachine.addState(&runResting);

  moving->addTransition(&checkArrived, resting);

  if (Serial) {
    Serial.println("Starting with iSL RPM STEPS_PER_ROTATION TURNS ANA_PIN MICROSTEPS_PER_STEP");
    Serial.println(sunsetLight, 4);
    Serial.println(TURNS);
    Serial.println(PHOTO_ANA_PIN);
    Serial.println(MICROSTEPS_PER_STEP);
  }

  #ifdef EEPROM_ADDRESS
    EEPROM.begin(512);
    float eeSunsetLight = 0.0;
    EEPROM.get(EEPROM_ADDRESS, eeSunsetLight);
    if (Serial) {
      Serial.print("Loading light from EEPROM: ");
      Serial.println(eeSunsetLight);
    }
    if (eeSunsetLight > 0.1 && eeSunsetLight < 0.9) {
      sunsetLight = eeSunsetLight;
    } else {
      if (Serial) {
        Serial.println("Loaded value out of range.");
      }
    }
  #endif


  // These may or may not be strictly necessary
  #ifdef A1_PIN
    pinMode(A1_PIN, OUTPUT);
    pinMode(A2_PIN, OUTPUT);
    pinMode(B1_PIN, OUTPUT);
    pinMode(B2_PIN, OUTPUT);
  #endif
  #ifdef STEP_PIN
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
  #endif
  #ifdef H1_PIN
    pinMode(H1_PIN, OUTPUT);
    pinMode(H2_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
  #endif

  pinMode(PH_PIN, INPUT_PULLUP);
  analogReadResolution(ANALOG_BITS);

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
  #ifdef LIFT_PIN4
    pinMode(LIFT_PIN4, OUTPUT);
    digitalWrite(LIFT_PIN4, 1);
  #endif
  #ifdef SINK_PIN1
    pinMode(SINK_PIN1, OUTPUT);
    digitalWrite(SINK_PIN1, 0);
  #endif

  #ifdef A1_PIN
    stepper.setSpeed(RPM);
    digitalWrite(A1_PIN, 0);
    digitalWrite(A2_PIN, 0);
    digitalWrite(B1_PIN, 0);
    digitalWrite(B2_PIN, 0);
  #endif
  #ifdef STEP_PIN
    float maxSpeed = (STEPS_PER_ROTATION * RPM * MICROSTEPS_PER_STEP / 60.f);
    stepper.setEnablePin(EN_PIN);
    stepper.setMinPulseWidth(40);  // The A4988 has minimum pulse width of 1us
    stepper.setMaxSpeed(maxSpeed);
    stepper.setAcceleration(4.f * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP);
    stepper.setPinsInverted(false, false, true);  // A4988 EN is active low
    stepper.disableOutputs();
  #endif
  #ifdef H1_PIN
    digitalWrite(H1_PIN, 0);
    digitalWrite(H2_PIN, 0);
    digitalWrite(EN_PIN, 0);
  #endif

  #ifdef M1_PIN
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    pinMode(M3_PIN, OUTPUT);
    #ifndef MICROSTEP_MODE
      digitalWrite(M1_PIN, 0);
      digitalWrite(M2_PIN, 0);
      digitalWrite(M3_PIN, 0);
    #else
      // microstep mode 0 (fullstep) gives 000, 1 (halfstep) gives 100, 2 gives 010, 3 gives 110, 4 gives 001, etc., except for the overrides below.
      bool m1mode = (MICROSTEP_MODE) % 2;
      bool m2mode = ((MICROSTEP_MODE) >> 1) % 2;
      bool m3mode = ((MICROSTEP_MODE) >> 2) % 2;
      #ifdef SIXTEENTH_STEP_IS_111  // Check the docs for the driver. The A988 is like this:
        if (MICROSTEP_MODE == 4) {
          m1mode = true;
          m2mode = true;
        }
      #endif
      #ifdef THIRTYTWOOTH_STEP_IS_III
        if (MICROSTEP_MODE == 5) {  // DRV8825 is like this when 1 2 and 3 are jumped because the PB2 is not working
          m2mode = true;
          m3mode = true;
        }
      #endif
      if (Serial) {
        Serial.print("M1 M2 M3: ");
        Serial.print(m1mode);
        Serial.print(m2mode);
        Serial.println(m3mode);
      }
      digitalWrite(M1_PIN, m1mode);
      digitalWrite(M2_PIN, m2mode);
      digitalWrite(M3_PIN, m3mode);
    #endif
  #endif

  #ifdef ENC1_PIN
    pinMode(ENC1_PIN, INPUT_PULLUP);
    pinMode(ENC2_PIN, INPUT_PULLUP);
    encoder.setEncoderHandler(rotationCallback);
  #endif

  #ifdef LED1_PIN
    digitalWrite(LED1_PIN, LED_OFF);
  #endif
  #ifdef NEOPIXEL_PIN
    led = CRGB::Black;
    FastLED.show();
  #endif
}

float s;
long stepstogo;
int ctr = 0;
void loop() {

  ctr++;

  #ifdef ENC1_PIN
    encoder.update();
  #endif

  #if STEP_PIN
    stepper.run();
    stateMachine.run();
  #endif

  if (lastReadTime < millis() - SLOW_INTERVAL) {
    s = sample();

    if (Serial) {
      Serial.println(s, 4);
    }
    lastReadTime = millis();

    if (isOpen && (s > sunsetLight + DAYLIGHT_MARGIN)) {  // Sunset happens
      if (Serial) {
        Serial.println(s, 4);
        Serial.println("Sunset");
      }
      myMoveTo(TURNS);
      isOpen = false;
    } else if (!isOpen && (s < sunsetLight - DAYLIGHT_MARGIN)) {  // Sunrise happens
      if (Serial) {
        Serial.println(s, 4);
        Serial.println("Sunrise");
      }
      myMoveTo(0);
      isOpen = true;
    }
  }

  #ifdef UP_PIN
    if (!digitalRead(UP_PIN) && !digitalRead(DOWN_PIN)) {
      #ifdef EEPROM_ADDRESS
        float target = s + isOpen ? DAYLIGHT_MARGIN/2.0 : 0-DAYLIGHT_MARGIN/2.0;
        if (Serial) {
          Serial.print("Setting to ");
          Serial.println(target, 4);
        }
        sunsetLight = target;

        float result = EEPROM.put(EEPROM_ADDRESS, target);
        if (Serial) {
          Serial.print("Writing light to EEPROM: ");
          Serial.println(sunsetLight);
          Serial.print("Result: ");
          Serial.println(result);
        }
        EEPROM.commit();
        #if LED1_PIN
          digitalWrite(LED1_PIN, 1);
          delay(500);
          digitalWrite(LED1_PIN, 0);
          delay(500);
          digitalWrite(LED1_PIN, 1);
          delay(500);
          digitalWrite(LED1_PIN, 0);
        #else
          #ifdef NEOPIXEL_PIN
            led = CRGB::Yellow;
            FastLED.show();
            delay(500);
            led = CRGB::Black;
            FastLED.show();
            delay(500);
            led = CRGB::Yellow;
            FastLED.show();
            delay(500);
            led = CRGB::Black;
            FastLED.show();
          #else
            delay(1500);
          #endif
        #endif
      #endif
    } else {
    if (digitalRead(UP_PIN) == 0) {
      continueFast(0.25);
    } else if (digitalRead(DOWN_PIN) == 0) {
      continueFast(-0.25);
    }

  #endif
}

void myMoveTo(long positionInTurns) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.moveTo(positionInTurns * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP);
  if (stateMachine.currentState == resting->index)
    stateMachine.transitionTo(moving);
}

bool continueFast(float rotations) {
  long current = stepper.currentPosition();

  stepper.moveTo(current + (MICROSTEPS_PER_STEP * STEPS_PER_ROTATION * rotations));
  stepper.setMaxSpeed(maxSpeed * BTN_FACTOR);
  if (stateMachine.currentState == resting->index)
    stateMachine.transitionTo(moving);
}

// Return the lightness value calculated with exponential moving average.
float sample() {
  float floatRead = ((float)analogRead(PHOTO_ANA_PIN)) / ANALOG_MAX;
  avgLight = (avgLight * AVG_TAIL) + (floatRead * AVG_HEAD);
  return avgLight;
}

#ifdef ENC1_PIN
void rotationCallback(EEncoder &enc) {
  position += enc.getIncrement();
}
#endif


void runMoving() {
  if (stateMachine.executeOnce) {
    stepper.enableOutputs();

    Serial.println("Move");
#ifdef NEOPIXEL_PIN
    if (stepper.distanceToGo() > 0) {
      led = CRGB::Green;
    } else {
      led = CRGB::Blue;
    }
    FastLED.show();
#endif
  }
}

void runResting() {
  if (stateMachine.executeOnce) {
    stepper.disableOutputs();
    Serial.println("Rest");
#ifdef NEOPIXEL_PIN
    led = CRGB::Black;
    FastLED.show();
#endif
  }
}

bool checkArrived() {
  return stepper.distanceToGo() == 0;
}
