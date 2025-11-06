#include <Arduino.h>
#include <Stepper.h>
#include <EEPROM.h>
#include <FastLED.h>

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
// #define SER
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
// #define LED_PIN 13
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
// #define LED_PIN 1
// #define DIR_PIN 2
// #define ANALOG_BITS 10

// For the Pico. Use JLink programmer.
// #define SER 
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
// #define LED_PIN 25
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
// #define SER 
// #define PH_PIN A0
// #define PHOTO_ANA_PIN A0 //  Analog pin 1 = PB2 on the
// #define UP_PIN 0
// #define DOWN_PIN 1
// #define A1_PIN 2
// #define A2_PIN 3
// #define B1_PIN 4
// #define B2_PIN 5
// #define LIFT_PIN1 6
// #define ANALOG_BITS 10

// For the RP2040-zero
// Board 'Raspberry Pi Pico/RP2040/RP2035' -> 'Waveshare RP2040 Zero'
// To enter bootloading mode, hold reset, hold boot, release reset, release boot
// #define SER 
#define BAUD 115200
#define PH_PIN 27
#define PHOTO_ANA_PIN 27 //  Analog pin 2 = PB28 on the pico
// #define UP_PIN 9
// #define DOWN_PIN 8
#define NEOPIXEL_PIN 16
#define EEPROM_ADDRESS 0
#define ANALOG_BITS 10
#define LIFT_PIN1 13
// // With DRV8825 or A988
#define EN_PIN 0 
#define M1_PIN 1
#define M2_PIN 4
#define M3_PIN 3
#define LIFT_PIN2 8
// #define LIFT_PIN3 5
#define STEP_PIN 6
#define DIR_PIN 7
#define MICROSTEP_MODE 3 // 2^X steps per step e.g. 3 = 8 microsteps per step

#define ANALOG_MAX ((1 << ANALOG_BITS) - 1)

// You may need to reduce AVG_HEAD if the motor starts at the wrong time of day
#define AVG_HEAD 0.3  // Weight of the latest sample
#define AVG_TAIL (1.0 - AVG_HEAD)

// Increase DAYLIGHT for day to start earlier / end later. Light increases conductivity in the photocell, which is connected to ground, so more light lowers the 
// voltage on the pin which is pulled up by a resistor 
// Upper limit (least light) is NOT 1.0. 
// On a trinket (5V):
//      Blk   Dark  Light
// 10K  3.2   3.17  1.0
// 75K  3.15  3.07  .99
//  1M  3.14  3.05  .84
#define INITIAL_SUNSET_LIGHT 0.5 // The analog reading value 0-1. This is the voltage divided by the logic voltage (3.3 or 5V).

// Increase DAYLIGHT_MARGIN if the blind reverses
// immediately after it opens / closes.
#define DAYLIGHT_MARGIN 0.01

// Customize these params according to your motor.
//#define STEPS_PER_ROTATION 64 * 2 * 64  // for the 28BYJ-48
#define STEPS_PER_ROTATION 200UL * 4 // for the JK28HS32-0674
#define RPM 20.
#define BTN_FACTOR 2.  // RPM is multiplied by this wnen button pressed

// Customize this param according to your blind.
#define TURNS 1.0//24.5  // Turns at sunset

bool isOpen = true;  // Make sure your blind is in this position when booting.
float avgLight = isOpen ? 0.0 : 0.8; // isOpen means more light, low value
float sunsetLight = INITIAL_SUNSET_LIGHT;

#ifdef A1_PIN
  Stepper stepper = Stepper(STEPS_PER_ROTATION, A1_PIN, A2_PIN, B1_PIN, B2_PIN);
#endif

#ifdef SER
  int readsSinceOut = 0;  // Count the number of reads since last output. Don't want to output every time.
#endif

#ifdef NEOPIXEL_PIN
  CRGB led;
#endif

void setup() {
  
  #ifdef LED_PIN
    pinMode(LED_PIN, OUTPUT);
    // digitalWrite(LED_PIN, 1);
    analogWrite(LED_PIN, 255);
  #endif

  #ifdef NEOPIXEL_PIN
    FastLED.addLeds<WS2812, NEOPIXEL_PIN>(&led, 1);
    led = CRGB::Red;
    FastLED.show();
  #endif

  #ifdef SER
    Serial.begin(BAUD);
    while (!Serial) {};
    Serial.println("Starting with iSL RPM STEPS_PER_ROTATION TURNS ANA_PIN");
    Serial.println(sunsetLight, 4);
    Serial.println(RPM);
    Serial.println(STEPS_PER_ROTATION);
    Serial.println(TURNS);
    Serial.println(PHOTO_ANA_PIN);
  #endif

  #ifdef EEPROM_ADDRESS
    EEPROM.begin(512);
    float eeSunsetLight = 0.0;
    EEPROM.get(EEPROM_ADDRESS, eeSunsetLight);
    Serial.print("Loading light from EEPROM: ");
    Serial.println(eeSunsetLight);
    if (eeSunsetLight > 0.1 && eeSunsetLight < 0.9) {
      sunsetLight = eeSunsetLight;
    } else {
      Serial.println("Loaded value out of range.");
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

    pinMode(PH_PIN, INPUT);
    analogReadResolution(ANALOG_BITS);;

  #ifdef UP_PIN
    pinMode(UP_PIN, INPUT);
    pinMode(DOWN_PIN, INPUT);
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

  #ifdef A1_PIN
    stepper.setSpeed(RPM);
    digitalWrite(A1_PIN, 0);
    digitalWrite(A2_PIN, 0);
    digitalWrite(B1_PIN, 0);
    digitalWrite(B2_PIN, 0);
  #else
    digitalWrite(EN_PIN, 1);    // High voltage disables the STSPIN220
    digitalWrite(STEP_PIN, 0);
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
      bool m1mode = (MICROSTEP_MODE) % 2;
      bool m2mode = ((MICROSTEP_MODE) >> 1) % 2;
      bool m3mode = ((MICROSTEP_MODE) >> 2) % 2;
      if (MICROSTEP_MODE == 4) { // Check the docs for the driver. The A988 is like this:
        m1mode = true;
        m2mode = true;
      }
      #ifdef SER
        Serial.print("M1 M2 M3: ");
        Serial.print(m1mode);
        Serial.print(m2mode);
        Serial.println(m3mode);
      #endif
      digitalWrite(M1_PIN, m1mode);
      digitalWrite(M2_PIN, m2mode);
      digitalWrite(M3_PIN, m3mode);
    #endif
  #endif
  #ifdef NEOPIXEL_PIN
    led = CRGB::Black;
    FastLED.show();
  #endif
}


void loop() {


  float s = sample();

  #ifdef SER
    if (readsSinceOut++ > 100) {
      Serial.println(s, 4);
      readsSinceOut = 0;
    }
  #endif

  if (isOpen && s > sunsetLight + DAYLIGHT_MARGIN) {  // Sunset happens
    #ifdef SER
      Serial.println(s, 4);
      Serial.println("Sunset");
    #endif
    turn(TURNS);                                   // Assuming positive rotations to close
    isOpen = false;
    delay(2000);
  } else if (!isOpen && s < sunsetLight - DAYLIGHT_MARGIN) {  // Sunrise happens
    #ifdef SER
      Serial.println(s, 4);
      Serial.println("Sunrise");
    #endif
    turn(-TURNS);                                          // Assuming negative rotation to open
    isOpen = true;
  } 

  #ifdef UP_PIN
    if (!digitalRead(UP_PIN) && !digitalRead(DOWN_PIN)) {
      Serial.print("Setting to ");
      Serial.println(sunsetLight, 4);

      sunsetLight = s;

      #ifdef EEPROM_ADDRESS
        float result = EEPROM.put(EEPROM_ADDRESS, sunsetLight);
        Serial.print("Writing light to EEPROM: ");
        Serial.println(sunsetLight);
        Serial.print("Result: ");
        Serial.println(result);

        EEPROM.commit();
      #endif
      #if LED_PIN
        digitalWrite(LED_PIN, 1);
        delay(500);
        digitalWrite(LED_PIN, 0);
        delay(500);
        digitalWrite(LED_PIN, 1);
        delay(500);
        digitalWrite(LED_PIN, 0);
      #endif

    } else {
      if (digitalRead(UP_PIN) == 0) {
        turn(.2, BTN_FACTOR);
      }
      if (digitalRead(DOWN_PIN) == 0) {
        turn(-.2, BTN_FACTOR);
      }
    }
  #endif

  delay(10);
}


// Return the lightness value calculated with exponential moving average.
float sample() {
  float floatRead = ((float)analogRead(PHOTO_ANA_PIN)) / ANALOG_MAX;
  avgLight = (avgLight * AVG_TAIL) + (floatRead * AVG_HEAD);
  return avgLight;
}


// A blocking implementation
void turn(double rotations, double factor) {
  #ifdef SER
    Serial.print("turning ");
    Serial.println(rotations);
  #endif
  #ifdef LED_PIN
    digitalWrite(LED_PIN, 1);
  #endif  
  #ifdef NEOPIXEL_PIN
    led = rotations > 0 ? CRGB::Green : CRGB::Blue;
    FastLED.show();
  #endif

  uint32_t microstepsPerStep = 1 << MICROSTEP_MODE;
  #ifdef A1_PIN
  
    stepper.setSpeed(RPM*factor * STEPS_PER_ROTATION * microstepsPerStep);
    stepper.step(-rotations * STEPS_PER_ROTATION);
    // Turn off the current because the stepper doesn't need to hold in place.
    digitalWrite(A1_PIN, 0);
    digitalWrite(A2_PIN, 0);
    digitalWrite(B1_PIN, 0);
    digitalWrite(B2_PIN, 0);
  #else
    digitalWrite(EN_PIN, 0);
    digitalWrite(DIR_PIN, rotations > 0);
    digitalWrite(STEP_PIN, 0);
    uint32_t microsteps = abs(rotations * STEPS_PER_ROTATION * microstepsPerStep);
    double delaymicros = 60000000./(RPM * STEPS_PER_ROTATION * microstepsPerStep*factor);
    #ifdef SER
      Serial.print("Microsteps:");
      Serial.println(microsteps);
      Serial.print("Delay");
      Serial.println(delaymicros);
    #endif
    for (uint32_t steps = 0; steps < microsteps; steps++) {
      digitalWrite(STEP_PIN, 1);
      delayMicroseconds(delaymicros/2);    // DRV8825 has 1.9 uS minimum pulse duration
      digitalWrite(STEP_PIN, 0);
      delayMicroseconds(delaymicros/2);
    }
    // delay(3000);
    // Turn off current
    digitalWrite(EN_PIN, 1);    // High voltage disables the STSPIN220
  #endif

  #ifdef LED_PIN
    digitalWrite(LED_PIN, 0);
  #endif
  
  #ifdef NEOPIXEL_PIN
    led = CRGB::Black;
    FastLED.show();
  #endif
}
void turn(double rotations) {
  turn(rotations, 1.0);
}