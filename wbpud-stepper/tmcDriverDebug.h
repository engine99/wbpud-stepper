
#include <TMCStepper.h>

void tmcDriverDebug(TMC2208Stepper tmcDriver) {
    uint32_t gconf = tmcDriver.GCONF();

      // Configurations in GCONF register
      Serial.print ("I_scale_analog: ");
      Serial.print (tmcDriver.I_scale_analog());
      Serial.println(" (Reset default=1) 0: Use internal reference derived from 5VOUT 1: Use voltage supplied to VREF as current reference");
      Serial.print("Internal_Rsense: ");
      Serial.print(tmcDriver.internal_Rsense());
      Serial.println(" (Reset default: OTP) 0: Operation with external sense resistors 1: Internal sense resistors. Use current supplied into VREF as reference for internal sense resistor. ");
      Serial.print("en_SpreadCycle: ");
      Serial.print(tmcDriver.en_spreadCycle());
      Serial.println(" (Reset default: OTP) 0: StealthChop PWM mode enabled (depending on velocity thresholds). Initially switch from off to on state while in stand still, only. 1: SpreadCycle mode enabled A high level on the pin SPREAD inverts this flag to switch between both chopper modes.");
      Serial.print("pdn_disable: ");
      Serial.print(tmcDriver.pdn_disable());
      Serial.println(" (Reset default: 0) 0: PDN_UART controls standstill current reduction 1: PDN_UART input function disabled. Set this bit, when using the UART interface!");
      Serial.print("mstep_reg_select: ");
      Serial.print(tmcDriver.mstep_reg_select());
      Serial.println(" (Reset default: 0) 0: Microstep resolution selected by MS1 and MS2 pins 1: Microstep resolution selected by MSTEP register");

      Serial.print("IFCNT raw: ");
      Serial.println(tmcDriver.IFCNT(), HEX);
      Serial.println(" (Incremented by the TMC2209 for each read access to the UART interface. The counter is reset to 0 after a write access to the UART interface.)");


      Serial.print("GSTAT raw: ");
      Serial.print(tmcDriver.GSTAT(), HEX);
      Serial.println(" (Re-Write with ‘1’ bit to clear respective flags) bit 0 (reset) 1: Indicates that the IC has been reset since the last read access to GSTAT. All registers have been cleared to reset values. bit 1 drv_err 1: Indicates, that the driver has been shut down due to overtemperature or short circuit detection since the last read access. Read DRV_STATUS for details. The flag can only be cleared when all error conditions are cleared. bit 2 uv_cp 1: Indicates an undervoltage on the charge pump. The driver is disabled in this case. This flag is not latched and thus does not need to be cleared.");

      // Should be -1 this time?
      Serial.print("GSTAT raw: ");
      Serial.print(tmcDriver.GSTAT(), HEX);
      Serial.println(" (Re-Write with ‘1’ bit to clear respective flags) bit 0 (reset) 1: Indicates that the IC has been reset since the last read access to GSTAT. All registers have been cleared to reset values. bit 1 drv_err 1: Indicates, that the driver has been shut down due to overtemperature or short circuit detection since the last read access. Read DRV_STATUS for details. The flag can only be cleared when all error conditions are cleared. bit 2 uv_cp 1: Indicates an undervoltage on the charge pump. The driver is disabled in this case. This flag is not latched and thus does not need to be cleared.");

      Serial.print("IFCNT raw: ");
      Serial.println(tmcDriver.IFCNT(), HEX);
      Serial.println(" (Incremented by the TMC2209 for each read access to the UART interface. The counter is reset to 0 after a write access to the UART interface.)");

      Serial.print("IOIN raw: ");
      Serial.println(tmcDriver.IOIN(), HEX);
      Serial.println(" INPUT (Reads the state of all input pins available) 0 ENN 1 0 2 MS1 3 MS2 4 DIAG 5 0 6 PDN_UART 7 STEP 8 SPREAD_EN 9 DIR 31.. 24 VERSION: 0x21=first version of the IC Identical numbers mean full digital compatibility.");


      Serial.print("IOIN version: ");
      Serial.println(tmcDriver.version(), HEX);
      Serial.print("GCONF raw: ");
      Serial.println(tmcDriver.GCONF(), HEX);
      // Serial.print("Enabled");
      // Serial.println(tmcDriver.isEnabled());

}