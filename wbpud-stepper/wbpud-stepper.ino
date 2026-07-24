#include <Arduino.h>
#include <AccelStepper.h>
#include <EEPROM.h>
#include <FastLED.h>
#include <EEncoder.h>
#include <StateMachine.h>
#include <momentary_button.h>
#include <TMC2209.h>
#include "tmcDriverDebug.h"
/**
* Arduino windowblind puller-upper-downer with a stepper motor. Opens a blind at sunrise, closes at sunset. Closes if full sun/over hot.
*
* Blinks:
* 2 -> Entering menu
* 3 -> Set sunrise/sunset brightness to current level
* 4 -> Set the limits. Use up/down to move blind to top/bottom. Press both to save.
* 5 -> Set the current. Use up/down to set. Press both to save.
* 6 -> Set the full=sun brightness and heat to current level.
* 7 -> Factory reset
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

// For the RP2040-zero
// Board 'Raspberry Pi Pico/RP2040/RP2035' -> 'Waveshare RP2040 Zero'
// To enter bootloading mode, hold reset, hold boot, release reset, release boot
// With the TMC2208/2209 Serial mode
#define BAUD 115200

#define PH_PIN        29         //A2 is pin 28 on the Tiny2040
#define PHOTO_ANA_PIN 29  // On some (Pi RP2040?) this should be the PB pin i.e. PH_PIN, otherwise it's the A pin number
#define NEOPIXEL_PIN  16
#define UP_PIN        10
#define DOWN_PIN      9
#define EEPROM_ADDY   0
#define ANALOG_BITS   12  // Tiny2040 is 12, RP2040 is 10
#define EN_PIN        8
#define STEP_PIN      2
#define DIR_PIN       1
#define RX_PIN        5
#define TX_PIN        4
#define SERIAL_PORT Serial2 // Use Serial2 (UART1) for TMC2208
#define R_SENSE 0.11f   // Current sense resistance in Ohms on TMC220X chips. Written on 2 resistors on the chip side.

#define ANALOG_MAX ((1 << ANALOG_BITS) - 1)

#define EEPROM_ADDRESS_SUNSET_LIGHT 0
#define EEPROM_ADDRESS_POSITION 1
#define EEPROM_ADDRESS_RANGE_TOP 2
#define EEPROM_ADDRESS_RANGE_BOTTOM 3
#define EEPROM_ADDRESS_FORCE_INDEX 4
#define EEPROM_ADDRESS_MIDDAY_LIGHT 5
#define EEPROM_ADDRESS_MIDDAY_TEMP 6
#define EEPROM_ADDRESS_STATE 7

// You may need to reduce AVG_HEAD if the motor starts at the wrong time of day
#define AVG_HEAD 0.2  // Weight of the latest sample
#define AVG_TAIL (1.0 - AVG_HEAD)

// Sunset darkness level. Increase DAYLIGHT for day to start earlier / end later. Light increases conductivity in the photocell, which is connected to ground, so more light lowers the
// voltage on the pin which is pulled up by a resistor
// Upper limit (least light) is NOT 1.0.
// On a trinket (5V):
//      Blk   Dark  Light
// 10K  3.2   3.17  1.0
// 75K  3.15  3.07  .99
//  1M  3.14  3.05  .84
#define SUNSET_LIGHT_INIT 0.6  // The analog reading value 0-1. This is the voltage divided by the logic voltage (3.3 or 5V). 0.5 good with a 10K. Initial / factory reset value.
#define MIDDAY_LIGHT_LEVEL_INIT 0.0 // 0 means incredible brightness, so midday close is disabled to start.
#define MIDDAY_TEMP_LEVEL_INIT 22 // 22 degress.
#define STATE_INIT 1 // initial state. 0 = night, 1 = day, 2 = closed for mid-day.

// Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT_MARGIN 0.02

//
// Motor params. Customize these params according to your motor.
//
// #define STEPS_PER_ROTATION 64 * 2 * 64  // for the 28BYJ-48
// #define STEPS_PER_ROTATION 200UL// * 4 // for the JK28HS32-0674
#define STEPS_PER_ROTATION 200UL  // for the 42STH34-0354A
#define MICROSTEP_MODE 1          // 2^X steps per step e.g. 3 = 8 microsteps per step // Check below for pin setting fixes
#define MICROSTEPS_PER_STEP (1 << MICROSTEP_MODE)
#define RPM 20
#define MANUAL_SPEED_FACTOR 1.2  // RPM is multiplied by this wnen button pressed
float forceLevels[] = {150, 200, 250, 300, 400, 500, 600, 700, 800, 900};  // in milliamps
#define FORCE_LEVEL_INIT 3 // Initial index into currentLevels (300mA)

// Customize this param according to your blind. (4cm per turn)
#define TOP_POSITION_INIT 0 // Turns at sunset = Bottom - Top
#define BOTTOM_POSITION_INIT 10 // 24 //-38 for office window.

// Button and light constants
#define BUTTON_DELAY 100  // milliseconds before upOnTick or downOnTick fires
#define BLINK_ON_DURATION 25
#define BLINK_OFF_DURATION 250
#define SLOW_INTERVAL 1000  // milliseconds between output, light sense ...

// Variables
short state;                   // Make sure your blind is in this position when booting. 0 = closed at night, 1 = open, 2 = closed for mid-day
float position;
float avgLight;
float sunsetLight;
float topPosition;
float bottomPosition;
short forceLevelIndex;
float middayLight;
float middayTemp;
long lastReadTime = 0;  // Count the number of reads since last output. Don't want to output every time.

// Controls the stepping timing over the motor driver's step and dir
#if defined(STEP_PIN) || defined(RX_PIN)
  AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
#endif

// For the TMCStepper. Configures the stepper driver over serial
#ifdef RX_PIN
  TMC2209 tmcDriver;
  HardwareSerial & tmc_serial = SERIAL_PORT;
#endif

#ifdef NEOPIXEL_PIN
  CRGB led;
#endif

#ifdef UP_PIN
  mt::MomentaryButton upButton(UP_PIN, mt::MomentaryButton::PinState::kHigh);
  mt::MomentaryButton downButton(DOWN_PIN, mt::MomentaryButton::PinState::kHigh);
#endif

// The motor state machine
StateMachine motorStateMachine;
State *moving, *resting;
float maxSpeed = (STEPS_PER_ROTATION * RPM * MICROSTEPS_PER_STEP / 60.f);

// The main state machine
StateMachine mainMachine;
State *running, *settingRange, *settingForce, *shortHolding, *mediumHolding, *longHolding, *xLongHolding, *xXLongHolding, *xXXLongHolding;
void runRunning();
void runSettingRange();
void runSettingForce();
void runShortHolding();
void runMediumHolding();
void runLongHolding();
void runXLongHolding();
void runXXLongHolding();
void runXXXLongHolding();

// Button states
bool upPressed; // True if the up button has been held longer than the delay time
bool downPressed; // True if the down button has been held longer than the delay time
bool upDelayPressed;
bool downDelayPressed;
bool bothPressed;
bool upOnTick;
bool upOffTick;
bool downOnTick;
bool downOffTick;
bool bothOnTick;
bool bothOffTick;
double menuEntryStamp;


// Boot procedure
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

  // Set up the motor state machine.
  motorStateMachine = StateMachine();
  moving = motorStateMachine.addState(&runMoving);
  resting = motorStateMachine.addState(&runResting);
  moving->addTransition(&checkArrived, resting);

  // Set up the main state machine.
  mainMachine = StateMachine();
  running = mainMachine.addState(&runRunning);
  settingRange = mainMachine.addState(&runSettingRange);
  settingForce = mainMachine.addState(&runSettingForce);
  shortHolding = mainMachine.addState(&runShortHolding);
  mediumHolding = mainMachine.addState(&runMediumHolding);
  longHolding = mainMachine.addState(&runLongHolding);
  xLongHolding = mainMachine.addState(&runXLongHolding);
  xXLongHolding = mainMachine.addState(&runXXLongHolding);
  xXXLongHolding = mainMachine.addState(&runXXXLongHolding);

  state = STATE_INIT;
  position = (state == 1 ? TOP_POSITION_INIT : BOTTOM_POSITION_INIT);
  sunsetLight = SUNSET_LIGHT_INIT;
  topPosition = TOP_POSITION_INIT;      
  bottomPosition = BOTTOM_POSITION_INIT;
  forceLevelIndex = FORCE_LEVEL_INIT;
  middayLight = MIDDAY_LIGHT_LEVEL_INIT;
  middayTemp = MIDDAY_TEMP_LEVEL_INIT;

  // Load and initialize sunset light from EEPROM if available. If not, use the default SUNSET_LIGHT.
  #ifdef EEPROM_ADDRESS_SUNSET_LIGHT
    EEPROM.begin(512);
    
    float eeSunsetLight = 0.0;
    EEPROM.get(EEPROM_ADDRESS_SUNSET_LIGHT, eeSunsetLight);

    if (Serial) {
      Serial.print("Loading light from EEPROM: ");
      Serial.println(eeSunsetLight);
    }

    if (eeSunsetLight > 0.1 && eeSunsetLight < 0.9) {
      if (Serial) Serial.println("Reading settings from storage.");
      sunsetLight = eeSunsetLight;
    
      EEPROM.get(EEPROM_ADDRESS_POSITION, position);
      EEPROM.get(EEPROM_ADDRESS_RANGE_TOP, topPosition);
      EEPROM.get(EEPROM_ADDRESS_RANGE_BOTTOM, bottomPosition);
      EEPROM.get(EEPROM_ADDRESS_FORCE_INDEX, forceLevelIndex);
      EEPROM.get(EEPROM_ADDRESS_MIDDAY_LIGHT, middayLight);
      EEPROM.get(EEPROM_ADDRESS_MIDDAY_TEMP, middayTemp);
      EEPROM.get(EEPROM_ADDRESS_STATE, state);
    } else {
      // Don't bother reading others. Write INIT values.
      if (Serial) Serial.println("Loaded value out of range. Initializing to factory settings.");
      EEPROM.write(EEPROM_ADDRESS_SUNSET_LIGHT, sunsetLight);
      EEPROM.write(EEPROM_ADDRESS_POSITION, position);
      EEPROM.write(EEPROM_ADDRESS_RANGE_TOP, topPosition);
      EEPROM.write(EEPROM_ADDRESS_RANGE_BOTTOM, bottomPosition);
      EEPROM.write(EEPROM_ADDRESS_FORCE_INDEX, forceLevelIndex);
      EEPROM.write(EEPROM_ADDRESS_MIDDAY_LIGHT, middayLight);
      EEPROM.write(EEPROM_ADDRESS_MIDDAY_TEMP, middayTemp);
      EEPROM.write(EEPROM_ADDRESS_STATE, state);
    }
  #endif

  avgLight = (state == 1 ? 0.0 : 0.9);  // State == 1 means open for day. Open means more light, low value


  // Log status on startup
  if (Serial) {
    Serial.print("Date updated:    ");
    Serial.println(__DATE__);

    Serial.print("State: ");
    Serial.println(state);
    Serial.print("Factory state: ");
    Serial.println(STATE_INIT);

    Serial.print("Sunset Light: ");
    Serial.println(sunsetLight, 4);
    Serial.print("Sunset Light INIT: ");
    Serial.println(SUNSET_LIGHT_INIT, 4);

    Serial.print("Position: ");
    Serial.println(position, 2);

    Serial.print("Top limit: ");
    Serial.println(topPosition, 2);
    Serial.print("Top limit INIT: ");
    Serial.println(TOP_POSITION_INIT, 2);

    Serial.print("Bottom limit: ");
    Serial.println(bottomPosition, 2);
    Serial.print("Bottom limit INIT: ");
    Serial.println(BOTTOM_POSITION_INIT, 2);
    
    Serial.print("Force index: ");
    Serial.print(forceLevelIndex);
    Serial.print(" (");
    Serial.print(forceLevels[forceLevelIndex]);
    Serial.println(")");
    Serial.print("Force index INIT: ");
    Serial.println(FORCE_LEVEL_INIT);

    Serial.print("Midday Light: ");
    Serial.println(middayLight, 4);
    Serial.print("Midday Light INIT: ");
    Serial.println(MIDDAY_LIGHT_LEVEL_INIT, 4);

    Serial.print("Midday Temp: ");
    Serial.println(middayTemp, 4);
    Serial.print("Midday Temp INIT: ");
    Serial.println(MIDDAY_TEMP_LEVEL_INIT, 4);

    Serial.print("RPM: ");
    Serial.println(RPM);
    
    Serial.print("Photo pin: ");
    Serial.println(PHOTO_ANA_PIN);
    Serial.print("Up pin: ");
    Serial.println(UP_PIN);
    Serial.print("Down pin: ");
    Serial.println(DOWN_PIN);
    Serial.print("EN pin: ");
    Serial.println(EN_PIN);

    Serial.print("Microsteps: ");
    Serial.println(MICROSTEPS_PER_STEP);
  }


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
    stepper.setMaxSpeed(maxSpeed);  // in steps per second
    stepper.setAcceleration(4.f * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP);
    stepper.setPinsInverted(false, false, true);  // A4988 EN is active low
    stepper.disableOutputs();
  #endif

  // When using TMC2208 or TMC2209 with Serial
  #ifdef RX_PIN 
    pinMode(TX_PIN, OUTPUT);
    pinMode(RX_PIN, INPUT);
    SERIAL_PORT.setTX(TX_PIN);
    SERIAL_PORT.setRX(RX_PIN);
    delay(50);
    tmcDriver.setup(tmc_serial, 115200);
    
    tmcDriver.setRunCurrent(50);
    // .begin();
    // tmcDriver.pdn_disable(true);                 // Use UART pins for config
    // tmcDriver.toff(5);                          // Enables driver in software
    // tmcDriver.mstep_reg_select(true);           // Microstep resolution selected by MSTEP register
    // tmcDriver.microsteps(MICROSTEPS_PER_STEP);
    // tmcDriver.rms_current(600);                 // Set mA current
    // tmcDriver.pwm_autoscale(true);              // Needed for stealthChop
    delay(50);
  #endif

  // When microstepping is set by hardware
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
  
  // Reset button event flags
  upOnTick = false;
  upOffTick = false;
  downOnTick = false;
  downOffTick = false;
  bothOnTick = false;
  bothOffTick = false;

  if (Serial) {
    if (tmcDriver.isSetupAndCommunicating()) {
      Serial.println("tmcDriver active");
    } else if (tmcDriver.isCommunicatingButNotSetup()) {
      Serial.println("tmcDriver communicating but not set up");
    }
  }
  
}

// Main loop
long upDelayStart; // Time when an up or down event first happened. up/downPressed isn't true and up/downOnTick doesn't fire until a short interval, so that both buttons may be pressed.
long downDelayStart;
void loop() {

  if (Serial) {
    
    //Serial.println(tmcDriver.getStallGuardResult());
    // Serial.print("mres ");
    // Serial.println(tmcDriver.mres());
    // Serial.print("pdn_disable ");
    // Serial.println(tmcDriver.pdn_disable());
  }

  // Reset the button transition states.
  upOnTick = false;
  upOffTick = false;
  downOnTick = false;
  downOffTick = false;
  bothOnTick = false;
  bothOffTick = false;

  // Detect button state transitions.
  #ifdef UP_PIN
    mt::MomentaryButton::ButtonState upEvent = upButton.DetectStateChange();
    mt::MomentaryButton::ButtonState downEvent = downButton.DetectStateChange();
    if (upEvent == mt::MomentaryButton::ButtonState::kPressed) {
      if (Serial) Serial.println("UpEventPressed");
      upDelayPressed = true;
      upDelayStart = millis();
      if (downDelayPressed) {
          bothOnTick = true;
          menuEntryStamp = millis();
      }
    } else if (upEvent == mt::MomentaryButton::ButtonState::kReleased) {
      if (Serial) Serial.println("UpReleased");  
      bothOffTick = true;
      upOffTick = true;
      upPressed = false;
      upDelayPressed = false;
      if (downPressed) {  // In case both were pressed when up is released, restart the down press, to give a moment for down to be release before 'Manual down'
        downPressed = false;
        downDelayPressed = true;
        downDelayStart = millis();
      }
    } else if (!upPressed && upDelayPressed && upDelayStart + BUTTON_DELAY < millis()) {
      if (Serial) Serial.println("UpPressed");
      upOnTick = true;
      upPressed = true;
    }

    if (downEvent == mt::MomentaryButton::ButtonState::kPressed) {
      if (Serial) Serial.println("downEventPressed");
      downDelayPressed = true;
      downDelayStart = millis();
      if (upDelayPressed) {
          bothOnTick = true;
          menuEntryStamp = millis();
      }
    } else if (downEvent == mt::MomentaryButton::ButtonState::kReleased) {
      if (Serial) Serial.println("DownReleased");
      bothOffTick = true;
      downOffTick = true;
      downPressed = false;
      downDelayPressed = false;
      if (upPressed) {  // In case both were pressed when down is released, restart the up press, to give a moment for up to be release before 'Manual up'
        upPressed = false;
        upDelayPressed = true;
        upDelayStart = millis();
      }
    } else if (!downPressed && downDelayPressed && downDelayStart + BUTTON_DELAY < millis()) {
      if (Serial) Serial.println("DownPressed");
      downOnTick = true;
      downPressed = true;
    }
  #endif  
  
  // Run the motor if needed
  motorStateMachine.run();

  // Run the blinker
  runBlinker();

  // Run the current program
  mainMachine.run();
}

// Motor helper. 0 is fully open, TURNS is fully closed.
void myMoveTo(long positionInTurns) {
  stepper.setMaxSpeed(maxSpeed);
  stepper.moveTo(positionInTurns * STEPS_PER_ROTATION * MICROSTEPS_PER_STEP);
  if (motorStateMachine.currentState == resting->index)
    motorStateMachine.transitionTo(moving);
}

// Turn on the moving indicator and advance this amount respecting the current acceleration.
void continueFast(float rotations) {
  long current = stepper.currentPosition();
  stepper.moveTo(current + (MICROSTEPS_PER_STEP * STEPS_PER_ROTATION * rotations));
  stepper.setMaxSpeed(maxSpeed * MANUAL_SPEED_FACTOR);
  if (motorStateMachine.currentState == resting->index)
    motorStateMachine.transitionTo(moving);
}

// Return the lightness value calculated with exponential moving average.
float sample() {
  float floatRead = ((float)analogRead(PHOTO_ANA_PIN)) / ANALOG_MAX;
  avgLight = (avgLight * AVG_TAIL) + (floatRead * AVG_HEAD);
  return avgLight;
}

//
// Motor state machine. Governs the acceleration, power state and LED indicator. 
//
void runMoving() {
  if (motorStateMachine.executeOnce) {
    stepper.enableOutputs();
    tmcDriver.enable();
    #ifdef NEOPIXEL_PIN
      if (stepper.distanceToGo() > 0) {
        led = CRGB::Green;
      } else {
        led = CRGB::Blue;
      }
      FastLED.show();
    #endif
  }
  stepper.run();
}

void runResting() {
  if (motorStateMachine.executeOnce) {
    if (Serial) Serial.println("Motor resting");    
    stepper.disableOutputs();
    tmcDriver.disable();
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
// LED Indicator
//
short blinksToGo = 0;
bool blinkOn = false;
long blinkStartStamp = 0;

// Start some blinks.
void blink(short blinks) {
  blinksToGo = blinks;
  blinkOn = false;
  blinkStartStamp = 0;
}

void runBlinker() {
  if (blinksToGo > 0) {
    if (!blinkOn && millis() - blinkStartStamp > BLINK_ON_DURATION + BLINK_OFF_DURATION) {  // Time to start a new blink-on
      #if LED1_PIN
        digitalWrite(LED1_PIN, 1);
      #endif
      #ifdef NEOPIXEL_PIN
        led = CRGB::Yellow;
        FastLED.show();
      #endif

      blinkOn = true;
      blinkStartStamp = millis();
    } else if (blinkOn && millis() - blinkStartStamp > BLINK_ON_DURATION) { // Time to end the blink-on
      #if LED1_PIN
        digitalWrite(LED1_PIN, 0);
      #endif
      #ifdef NEOPIXEL_PIN
        led = CRGB::Black;
        FastLED.show();
      #endif

      blinkOn = false;
      blinksToGo--;
    }
  }
}


//
// Main programs
//
void runRunning() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("runRunning");
  }

  if (bothOnTick) {
    mainMachine.transitionTo(shortHolding);
  } else if (upPressed && !downPressed) {
    //if (Serial) Serial.println("Manual up");
    continueFast(0.05);
  } else if (downPressed && !upPressed) {
    //if (Serial) Serial.println("Manual down");
    continueFast(-0.05);  
  } else {
    // Check for sunrise/sunset.
    if (lastReadTime < millis() - SLOW_INTERVAL) {
      float s = sample();

      if (Serial) {
        Serial.println(s, 4);
        //Serial.println(tmcDriver.SG_RESULT());
      }
      lastReadTime = millis();

      if (state == 1 && (s > sunsetLight + DAYLIGHT_MARGIN)) {  // Sunset happens
        if (Serial) {
          Serial.println(s, 4);
          Serial.println("Sunset");
        }
        myMoveTo(bottomPosition);
        state = 0;
      } else if (state == 0 && (s < sunsetLight - DAYLIGHT_MARGIN)) {  // Sunrise happens
        if (Serial) {
          Serial.println(s, 4);
          Serial.println("Sunrise");
        }
        myMoveTo(topPosition);
        state = 1;
      }
    }
  }
}

void runSettingRange() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("Setting Range");
  }
  if (bothOnTick) {
    upDelayPressed = false;
    downDelayPressed = false;
    mainMachine.transitionTo(running);
    blink(4);
  } else {
    if (upPressed && !downPressed) {
    if (Serial) Serial.println("Ranging up");
      continueFast(0.05);
      topPosition = stepper.currentPosition();
    } else if (downPressed && !upPressed) {
    if (Serial) Serial.println("Ranging down");
      continueFast(-0.05);
      bottomPosition = stepper.currentPosition();
    } else if (upOffTick || downOffTick) {
      // Persist the range
      if (Serial) Serial.println("Persisting range");
      #ifdef EEPROM_ADDRESS_RANGE_TOP
        EEPROM.put(EEPROM_ADDRESS_RANGE_TOP, topPosition);
        EEPROM.put(EEPROM_ADDRESS_RANGE_BOTTOM, bottomPosition);
      #endif
    }
  }
}

void setForceLevel() {
  float forceLevel = forceLevels[forceLevelIndex];
  if (Serial) {
    Serial.print("Setting force to: ");
    Serial.println(forceLevel);
  }
  #ifdef RX_PIN
    //tmcDriver.rms_current(forceLevel);  // Set mA current
  #endif
}

void runSettingForce() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("Setting Force");
  }
  if (bothOnTick) {
    upDelayPressed = false;
    downDelayPressed = false;
    mainMachine.transitionTo(running);
    blink(5);
  } else if (upOnTick) {
    // Increase force
    if (forceLevelIndex < std::size(forceLevels) - 1) {
      forceLevelIndex++;
      setForceLevel();
    }
  } else if (downOnTick) {
    // Decrease force
    if (forceLevelIndex > 0) {
      forceLevelIndex--;
      setForceLevel();
    }
  }
}

void runShortHolding() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("Short holding - hold to enter menu");
    blink(2);
    menuEntryStamp = millis();
  }
  
  if (bothOffTick) {     // Released when holding < 5 seconds: do nothing and go back to running program.
    if (Serial) Serial.println("Menu exit released");
    mainMachine.transitionTo(running);
  } else if (millis() - menuEntryStamp > 5000) {
    mainMachine.transitionTo(mediumHolding);
  }
}

void runMediumHolding() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("Medium holding - release to set daylight");
    blink(3);
  }

  if (bothOffTick) {     // Released when holding > 5 seconds: set daylight.
    // Set Daylight
    if (Serial) Serial.println("Setting daylight");
    mainMachine.transitionTo(running);
    blink(3);
  } else if (millis() - menuEntryStamp > 10000) {
    mainMachine.transitionTo(longHolding);
  }
}

void runLongHolding() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("Long holding - release to enter 'Set range' mode");
    blink(4);
  }

  if (bothOffTick) {     // Released when holding > 10 seconds: go to settingRange program.
    mainMachine.transitionTo(settingRange);
  } else if (millis() - menuEntryStamp > 15000) {
    mainMachine.transitionTo(xLongHolding);
  }
}


void runXLongHolding() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("X long holding - release to enter 'Set force' mode");
    blink(5);
  }

  if (bothOffTick) {     // Released when holding > 15 seconds: go to settingForce program.
    mainMachine.transitionTo(settingForce);
  } else if (millis() - menuEntryStamp > 20000) {
    mainMachine.transitionTo(xXLongHolding);
  }
}


void runXXLongHolding() {
  if (mainMachine.executeOnce) {
    if (Serial) Serial.println("XX long holding - release to set overheat threshold");
    blink(6);
  }

  if (bothOffTick) {     // Released when holding > 20 seconds: set overheat threshold.
    // set overheat threshold
    Serial.println("Setting overheat threshold");
    mainMachine.transitionTo(running);            
  } else if (millis() - menuEntryStamp > 25000) {
    mainMachine.transitionTo(xXXLongHolding);
  }
}


void runXXXLongHolding() {
  if (mainMachine.executeOnce) {
    Serial.println("XXX long holding - release to factory reset");
    blink(7);
  }

  if (bothOffTick) {     // Released when holding > 25 seconds: factory reset
    // Factory reset
    Serial.println("Factory reset");
    blink(10);
    mainMachine.transitionTo(running);            
  } else if (millis() - menuEntryStamp > 30000) {    // Held 30 seconds: return to shortHolding where user may exit menu.
    mainMachine.transitionTo(shortHolding);
  }
}