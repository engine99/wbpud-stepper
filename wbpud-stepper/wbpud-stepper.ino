#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <EEncoder.h>
#include <StateMachine.h>
#include <momentary_button.h>
#include "blinker.h"
#include <TMCStepper.h>
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
// #define UP_PIN 28
// #define DOWN_PIN 27
#define EEPROM_ADDRESS 0
#define ANALOG_BITS 12  // Tiny2040 is 12, RP2040 is 10
// // With the A988
// #define SIXTEENTH_STEP_IS_111 1
// #define EN_PIN 7
// #define M1_PIN 6
// #define M2_PIN 5
// #define M3_PIN 4
// #define LIFT_PIN2 3
// #define LIFT_PIN3 2
// #define STEP_PIN 1
// #define DIR_PIN 0

// With the TMC2208/2209 Serial mode
#define EN_PIN      8
#define STEP_PIN    2
#define DIR_PIN     1
#define RX_PIN      5
#define TX_PIN      4
#define SERIAL_PORT Serial2 // Use Serial2 (UART1) for TMC2208
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f   // Current sense resistance in Ohms on TMC220X chips. Written on 2 resistors on the chip side.

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
#define SUNSET_LIGHT 0.6  // The analog reading value 0-1. This is the voltage divided by the logic voltage (3.3 or 5V). 0.5 good with a 10K

// Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT_MARGIN 0.02

// Customize these params according to your motor.
//#define STEPS_PER_ROTATION 64 * 2 * 64  // for the 28BYJ-48
// #define STEPS_PER_ROTATION 200UL// * 4 // for the JK28HS32-0674
#define STEPS_PER_ROTATION 100UL  // for the 42STH34-0354A
#define MICROSTEP_MODE 4          // 2^X steps per step e.g. 3 = 8 microsteps per step // Check below for pin setting fixes
#define MICROSTEPS_PER_STEP (1 << MICROSTEP_MODE)

#define RPM 20
// Customize this param according to your blind. (4cm per turn)
#define TURNS 24 //-38  // Turns at sunset
#define BTN_FACTOR 1.2  // RPM is multiplied by this wnen button pressed

// Constant constants
#define MEDIUM_HOLD 3000 // milliseconds holding buttons
#define LONG_HOLD 8000 // milliseconds holding buttons
#define FLASH_ON_DURATION 25
#define FLASH_OFF_DURATION 250
#define SLOW_INTERVAL 1000  // milliseconds between output, light sense ...

// Variables
bool isOpen = true;                   // Make sure your blind is in this position when booting.;
float avgLight = isOpen ? 0.0 : 0.8;  // isOpen means more light, low value
float sunsetLight = SUNSET_LIGHT;
long lastReadTime = 0;  // Count the number of reads since last output. Don't want to output every time.
StateMachine stateMachine;
State *moving, *resting;
float maxSpeed = (STEPS_PER_ROTATION * RPM * MICROSTEPS_PER_STEP / 60.f);

// For drivers like the A4988, DRV8825, STSPIN220
#if defined(STEP_PIN) || defined(RX_PIN)
  AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
#endif

// For the TMCStepper
#ifdef RX_PIN
  TMC2208Stepper tmcDriver(RX_PIN, TX_PIN, R_SENSE);
#endif

#if defined(LED_PIN) || defined(NEOPIXEL_PIN)
  StateMachine ledStateMachine;
  State *dark, *blinking3, *blinking8;
  Blinker *blinker3, *blinker8;
#endif

#ifdef NEOPIXEL_PIN
  CRGB led;
#endif

#ifdef UP_PIN
  mt::MomentaryButton upButton(UP_PIN, mt::MomentaryButton::PinState::kHigh);
  mt::MomentaryButton downButton(DOWN_PIN, mt::MomentaryButton::PinState::kHigh);
  StateMachine buttonStateMachine;
  State *notHolding, *holding, *mediumHolding, *longHolding;
#endif

void setup() {
  delay(2000);
  Serial.begin(BAUD);
  
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
    Serial.println("Starting with iSL TURNS RPM ANA_PIN MICROSTEPS_PER_STEP");
    Serial.println(sunsetLight, 4);
    Serial.println(TURNS);
    Serial.println(RPM);
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
  #ifdef STEP_PIN
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
  #endif

  pinMode(PH_PIN, INPUT_PULLUP);
  analogReadResolution(ANALOG_BITS);

  #ifdef UP_PIN
    pinMode(UP_PIN, INPUT_PULLUP);
    pinMode(DOWN_PIN, INPUT_PULLUP);
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

  #ifdef RX_PIN // for the TMC2208
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT);
    SERIAL_PORT.setTX(TX_PIN);
    SERIAL_PORT.setRX(RX_PIN);
    SERIAL_PORT.begin(19200);
    delay(100);
    tmcDriver.begin();
    tmcDriver.pdn_disable(true);                 // Use UART pins for config
    tmcDriver.toff(5);                          // Enables driver in software
    tmcDriver.mstep_reg_select(true);           // Microstep resolution selected by MSTEP register
    tmcDriver.microsteps(MICROSTEPS_PER_STEP);
    tmcDriver.rms_current(300);                 // Set mA current
    tmcDriver.pwm_autoscale(true);              // Needed for stealthChop
  #endif

  #ifdef M1_PIN
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    #ifdef M3_PIN
      pinMode(M3_PIN, OUTPUT);
    #endif
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
      #ifdef M3_PIN
        digitalWrite(M3_PIN, m3mode);
      #endif
    #endif
  #endif

  #ifdef LED1_PIN
    digitalWrite(LED1_PIN, LED_OFF);
  #endif
  #ifdef NEOPIXEL_PIN
    led = CRGB::Black;
    FastLED.show();
  #endif
  
  #if defined(LED1_PIN) || defined(NEOPIXEL_PIN)
    ledStateMachine = StateMachine();

    blinker3 = new Blinker(3, FLASH_ON_DURATION, FLASH_OFF_DURATION, &flashOn, &flashOff, &goDark);
    blinker8 = new Blinker(8, FLASH_ON_DURATION, FLASH_OFF_DURATION, &flashOn, &flashOff, &goDark);

    dark = ledStateMachine.addState(&runNothing);
    blinking3 = ledStateMachine.addState(&runBlinker3);
    blinking8 = ledStateMachine.addState(&runBlinker8);
  #endif

  #if UP_PIN
    buttonStateMachine = StateMachine();
    notHolding = buttonStateMachine.addState(&runNothing);
    holding = buttonStateMachine.addState(&runHolding);
    mediumHolding = buttonStateMachine.addState(&runMediumHolding);
    longHolding = buttonStateMachine.addState(&runLongHolding);
    holding->addTransition(&checkMediumHold, mediumHolding);
    mediumHolding->addTransition(&checkLongHold, longHolding);
  #endif
}

float s;
long stepstogo;
int ctr = 0;
void loop() {

  ctr++;

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
    runButtons();
  #endif

  #if defined(LED1_PIN) || defined(NEOPIXEL_PIN)
    ledStateMachine.run();
  #endif
}

void myMoveTo(long positionInTurns) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.moveTo(positionInTurns * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP);
  if (stateMachine.currentState == resting->index)
    stateMachine.transitionTo(moving);
}

// Set light to Moving and advance this amount respecting the current acceleration.
void continueFast(float rotations) {
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
    if (Serial) {
     
      Serial.println("Motor resting");
    }
    stepper.disableOutputs();
    #ifdef NEOPIXEL_PIN
      led = CRGB::Black;
      FastLED.show();
    #endif
  }
}

bool checkArrived() {
  return stepper.distanceToGo() == 0;
}


//
// Buttons
//
double doubleHoldStamp = 0;
bool downPressed = false;
bool upPressed = false;

// Check and respond to button presses/releases
void runButtons() {
  #ifdef UP_PIN
    mt::MomentaryButton::ButtonState upEvent = upButton.DetectStateChange();
    mt::MomentaryButton::ButtonState downEvent = downButton.DetectStateChange();

    if (upEvent == mt::MomentaryButton::ButtonState::kPressed) {
      upPressed = true;
      if (downPressed) {
        buttonStateMachine.transitionTo(holding);
      }
    } else if (upEvent == mt::MomentaryButton::ButtonState::kReleased) {
      upPressed = false;
      onRelease();
    }

    if (downEvent == mt::MomentaryButton::ButtonState::kPressed) {
      downPressed = true;
      if (upPressed) {
        buttonStateMachine.transitionTo(holding);      
      }
    } else if (downEvent == mt::MomentaryButton::ButtonState::kReleased) {
      downPressed = false;
      onRelease();      
    }
    
    if (upPressed && !downPressed) {
      if (Serial) {
        Serial.println("Going up");
      }
      continueFast(0.05);    
    } else if (downPressed && !upPressed) {
      if (Serial) {
        Serial.println("Going down");
      }
      continueFast(-0.05);
    } else {
      buttonStateMachine.run();
    }
  #endif
}

void runNothing() {}

#ifdef UP_PIN
  void runHolding() {
    if (buttonStateMachine.executeOnce) {
      Serial.println("runHolding");
      doubleHoldStamp = millis();
    }

  }

  bool checkMediumHold() { return millis() - doubleHoldStamp > MEDIUM_HOLD;}

  bool checkLongHold() { return millis() - doubleHoldStamp > LONG_HOLD;}

  void runMediumHolding() {
    if (buttonStateMachine.executeOnce) {
      onMediumHold();
    }
    
  }

  void runLongHolding() {
    if (buttonStateMachine.executeOnce) {
      onLongHold();
    }
  }

  void onMediumHold() {
    ledStateMachine.transitionTo(blinking3);
  }

  void onRelease() {
    
    if (buttonStateMachine.currentState == mediumHolding->index) {
      Serial.println("Setting luminance");
      ledStateMachine.transitionTo(blinking3);
    }
    buttonStateMachine.transitionTo(notHolding);
  }

  void onLongHold() {
    Serial.println("onLongHold");
    ledStateMachine.transitionTo(blinking8);
    Serial.println("Resetting range and light ...");
    //Reset
  }
#endif

//
// LED Indicator
//
void runBlinker3() {
  blinker3->run();
}

void runBlinker8() {
  blinker8->run();
}

void flashOn() {
  #if LED1_PIN
    digitalWrite(LED1_PIN, 1);
  #endif

  #ifdef NEOPIXEL_PIN
    led = CRGB::Yellow;
    FastLED.show();
  #endif 
}

void flashOff() {
  #if LED1_PIN
    digitalWrite(LED1_PIN, 0);
  #endif

  #ifdef NEOPIXEL_PIN
    led = CRGB::Black;
    FastLED.show();
  #endif 
}

void goDark() {
  Serial.println("goDark");
  ledStateMachine.transitionTo(dark);
}

