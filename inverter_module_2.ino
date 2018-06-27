/*
 To do: search for SYS_ENABLE and make sure all calls setting it to zero
 have OCR1B set to 0 and a delay first.  Same as for halt/reset.

 To do: when reset command is received, re-read the unit ID.  Found a case
 when the unit powered up with the wrong ID!  At least if it is re-read
 after performing a manual reset normality will resume.

 Solar Inverter V2.01 - current-fed full bridge with active clamp design
 
 Written by Martin van den Nieuwelaar, martin at gadgets.co.nz June 2015.
 Released under the Creative Commons Attribution Share-Alike 3.0 license.
 Absolutely no warranty etc. etc.

 User functions:
 
 DIP-1 through DIP-4 set the unit ID 0-15.  DIP-1 is MSB, DIP-4 LSB.
  
 Arduino wiring:

 A0  Panel current measurement
 A1  Panel voltage measurement
 A2  Fan pulse (input, active low)
 A3  Output capacitor voltage; 405V -> 2V (input)

 A4  DIP-1 (input)
 A5  DIP-2 (input)
 A6  DIP-3 (input)
 A7  DIP-4 (input)
 
 D2  Over voltage (input, active low); output cap has > ~310V
 D3  SYNC (input) - NO LONGER USED
 D4  SYS-VIA (stand-alone (no master controller) only; output at ~50Hz)
 D5  SYS-VIB (stand-alone (no master controller) only; output at ~50Hz)
 D6  FAN-DRIVE (output)
 D7  1-WIRE interface; likely to be used for 18B20 temperature sensor
 D8  SYS-ENABLE (output)
 D9  SYS-OUTPUT (output); enable AC output by energising output relay
 D10 SYS-DRIVE (output at ~25kHz)
 D11 SYS-PWM (output at ~50Hz) - NO LONGER USED - MUST BE HIGH FOR CURRENT HARDWARE
 D12 -unused-
 D13 SYS-FAULT (output, active low); fault detected
 
 SYS-DRIVE: This signal is split with alternate pulses being inverted and each
 driving a pair of transistors.  The upshot is that each output
 is normally high, except for the duration that SYS-DRIVE is high, when
 alternately each of the outputs is low.  So when SYS-DRIVE is high the
 load is connected and when SYS-DRIVE is low boost mode is active.
 */

#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Send useful information to the console
//#define DEBUG

// Address to store the maximum limited power output from a module.
// The value in the address is actually scaled 2 x to get the real
// power output.  So theoretically 0-255 -> 0-511 Watts.
#define EEPROM_ADDRESS_MAX_POWER 0

// If this address contains a non-zero value the module should
// go into halted mode.  This is used to take modules off-line,
// perhaps for days at a time.
#define EEPROM_ADDRESS_HALTED 1

// Limit power to this to prevent possible meltdown, (in Watts)
#define MAX_POWER 425

// Limit the lower bound for the maximum module power output the user can specify
#define MIN_MAX_POWER 50

// Module ID.  Normally commented out which will cause the value to be
// read from DIP switches
//#define ID 0

// PWM output is on pin 10.  Pin 10 supports 16 bit Timer1 and finer granularity PWM!
#define SYS_DRIVE 10

// No longer used
#define SYS_PWM 11

// No longer used
#define SYNC 3

// If we are to be the generator for the clocking signals for the AC output
// we will use these pins
#define SYS_VIA 4
#define SYS_VIB 5

// If this goes low, the output cap has > ~310V on it
#define SYS_OVER 2

// Enable the generation of high frequency (~25kHz) pulses to the transformer
#define SYS_ENABLE 8

// Enable the output relay
#define SYS_OUTPUT 9

// There is an unresolvable fault; active low
#define SYS_FAULT 13

#define SENSE_CURRENT A0
#define SENSE_VOLTAGE A1
#define SENSE_CAP A3

// Pin to use for the 1-wire interface
#define ONE_WIRE 7

// Fan output control; active HIGH
#define FAN_DRIVE 6

// Detection of fan pulses
#define FAN_PULSE A2

// Arduino built-in voltage reference in Volts; we're using VCC, so 5.0V
#define VREF 5.0

// Resistor values on voltage divider network used for measuring
// source voltage
#define R1 100000
#define R2 6800

// DIP switch is connected on analogue inputs
#define DIP_1 A4
#define DIP_2 A5
#define DIP_3 A6
#define DIP_4 A7

// The top value for the 16-bit timer.  Dictates the PWM frequency.
// Using 320 gives 50k interrupts/second or 25kHz at the output of
// the converter as it's push/pull - one interrupt the load is
// driven with one polarity, the next interrupt the polarity is
// reversed.
#define TOP 325

// The length of the 'mains' waveform generated by the inverter
#define MAINS_SEQUENCE_LENGTH 30

// Maximum length of commands from the user/master controller
#define LINE_LENGTH 10

// Possible error codes
#define ERROR_NONE 0
#define ERROR_UNDER_TEMP 1
#define ERROR_OVER_TEMP 2
#define ERROR_UNDER_CURRENT 3
#define ERROR_OVER_CURRENT 4
#define ERROR_CAP_VOLTAGE_LOW 5
#define ERROR_CAP_VOLTAGE_HIGH 6

// Minimum and maximum temperatures for operation; anything outside this
// range is considered a fault
#define MIN_TEMPERATURE -30.0
#define MAX_TEMPERATURE 85.0

// Switching temperatures for the (optional) fan
#define FAN_ON_TEMP 40.0
#define FAN_OFF_TEMP 35.0

// States of the finite state machine for representing what mode
// the converter is operating in
enum {S0 = 0, S1, S2, S3, S4, S5};

// For parsing command line interface
char cli[LINE_LENGTH];
uint8_t cli_pos = 0;

// We make a simple pseudo sine waveform with the inverter
uint8_t mains_pos[] = {1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t mains_neg[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0};
volatile uint8_t mains_counter = 0;

uint8_t id; // Module ID
volatile uint16_t pwm; // Current PWM value
uint8_t state; // Finite State Machine

// In order to perform rate limiting we Keep track of time when events occurred
unsigned long timeout_pause = millis();
unsigned long timeout_temp = 0;

OneWire ds(ONE_WIRE); // Just a DS18B20 temp sensor.
DallasTemperature temp_sensor(&ds);
volatile bool flag_over = false; // Used to notify of an over-voltage situation

int error_code = ERROR_NONE; // No errors to start with

uint16_t max_power; // Maximum module power as configured by user

// Interrupt handling rountine called when the voltage on the output
// capacitor reaches operational levels (~310V).  At this point the
// inverter can be started.

void over_int() {
  flag_over = true;
  
  return;
}


// Called 30 times every 50Hz cycle; 1500Hz or every 666us.
// A PCB jumper dictates whether SYS_VIA/SYS_VIB drive
// the inverter or the ISO_VIA/ISO_VIB lines from the
// master controller drive the inverter.  The former is
// used in stand-alone mode, the latter when multiple
// inverters are connected in parallel (so they don't fight
// each other).

ISR(TIMER2_OVF_vect) {
  digitalWrite(SYS_VIA, mains_pos[mains_counter]);
  digitalWrite(SYS_VIB, mains_neg[mains_counter]);
  
  if (++mains_counter == MAINS_SEQUENCE_LENGTH)
    mains_counter = 0;
}


void setup(void) {
  // DIP switches are inputs with pull-ups enabled
  pinMode(DIP_1, INPUT_PULLUP);
  pinMode(DIP_2, INPUT_PULLUP);
  pinMode(DIP_3, INPUT_PULLUP);
  pinMode(DIP_4, INPUT_PULLUP);
  
  digitalWrite(SYS_VIA, 0);
  pinMode(SYS_VIA, OUTPUT);
  
  digitalWrite(SYS_VIB, 0);
  pinMode(SYS_VIB, OUTPUT);
  
  digitalWrite(FAN_DRIVE, 0);
  pinMode(FAN_DRIVE, OUTPUT);
    
  digitalWrite(SYS_OUTPUT, 0);
  pinMode(SYS_OUTPUT, OUTPUT);
  
  // No longer used, but must be high for the current version hardware
  digitalWrite(SYS_PWM, 1);
  pinMode(SYS_PWM, OUTPUT);
 
  digitalWrite(SYS_ENABLE, 0);
  pinMode(SYS_ENABLE, OUTPUT);
 
  // No faults for now; remember it's active low
  digitalWrite(SYS_FAULT, 1);
  pinMode(SYS_FAULT, OUTPUT);
    
  // start serial port
  Serial.begin(9600); // 9600 bps

  noInterrupts();
  
  // We are going to use Timer 1 the 16 bit timer for the PWM
  
  TCCR1A = 0;
  TCCR1B = 0;
  
  // COM1B1 and COM1B0 set means set OC1B on compare match,
  // clear OC1B at BOTTOM.
  // By not setting COM1A1 or COM1A0 we leave OC1A alone.
  // OC1A is on pin D9.
  // OC1B is on pin D10.
  //
  // Setting WGM13, WGM12, WGM11 and WGM10 gives
  // fast PWM from BOTTOM to OCR1A
  //
  // Clock select.
  // Setting CS10 and clearing CS11 and CS12 means div by 1
  // or no pre-scalar
  
  TCCR1A |= _BV(COM1B1) | _BV(WGM10) | _BV(WGM11) | _BV(COM1B0);
  TCCR1B |= _BV(CS10) | _BV(WGM12) | _BV(WGM13); // div by 1 (16MHz)

  // The TOP value that TCNT1 counts up to goes into OCR1A
  OCR1A = TOP;

  // The 'trigger' point goes into OCR1B.
  // We have inverted the output of OC1B in order that
  // we can get down to 0% duty cycle (but not quite up
  // to 100%).
  //
  // Setting to TOP gives 0V output
  // Setting to TOP-1 gives tiny duty cycle output
  // ...
  // Setting to 0 gives almost 100% duty cycle
  
  // Use these values as maximum otherwise flip flop doesn't flip flop
  
  OCR1B = 0;
  pwm = 0;


  // We are going to use Timer 2, the 8 bit timer, for the '50Hz' generator
  // that runs at 30 times per cycle or 1500Hz (0.6666ms)
  // This is only connected to the inverter when running in stand-alone
  // mode when the relevant PCB jumper is set.
  
  TCCR2A = 0;
  TCCR2B = 0;
  
  // Setting WGM22, WGM21 and WGM20 gives
  // fast PWM from BOTTOM to OCR2A
  
  // Clock select.
  // Setting CS22 means div by 64 (250000 ticks/s or 4us per tick)
  
  TCCR2A |= _BV(WGM20) | _BV(WGM21);
  TCCR2B |= _BV(CS22) | _BV(WGM22); // div by 64 (250kHz)

  // The interrupt mask register, TIMSKx
  TIMSK2 |= _BV(TOIE2); // Interrupt called on timer overflow

  // The TOP value that TCNT2 counts up to goes into OCR2A.
  // Using 167 gives an interrupt every 668us (1500Hz).
  OCR2A = 167; // This is the value of TOP
  
  // We don't bother setting OCR2B because we're not doing anything
  // based on it.

  interrupts();
  
  // System reaching operational voltage causes an interrupt
  // so we can quickly do something about it!

  pinMode(SYS_OVER, INPUT);
  attachInterrupt(0, over_int, FALLING);

  // Module ID number is read from DIP switch settings

#ifdef ID
  id = ID;
#else
  id = (!digitalRead(DIP_1)<<3) +
       (!digitalRead(DIP_2)<<2);

  // Bug, and workaround!
  // ADC6 and ADC7 cannot support digital inputs!
  // Use analogue inputs instead!
  // Seem to float around 450...

  if (analogRead(DIP_3) < 5)
    id += 2;
  if (analogRead(DIP_4) < 5)
    id += 1;
#endif

#ifdef DEBUG
  Serial.print("Set to ID ");
  Serial.println(id);
#endif

  max_power = EEPROM.read(EEPROM_ADDRESS_MAX_POWER)*2;

  // Limit the lower power limit that the user can specify
  
  if (max_power < MIN_MAX_POWER)
    max_power = MIN_MAX_POWER;

#ifdef DEBUG
  Serial.print("User-defined maximum power limit ");
  Serial.print(max_power);
  Serial.println(" Watts");
#endif

  // If NVRAM says we are halted, we should go into the halted state
  
  if (EEPROM.read(EEPROM_ADDRESS_HALTED))
    state = S5; // Halted state
  else
    state = S0; // Start state
 
  pinMode(SYS_DRIVE, OUTPUT); // Set DDR _after_ setting OCR1x value
                              // Refer section 16.7.3 of Atmel datasheet
                                
  temp_sensor.begin();
  temp_sensor.setResolution(9); // 9 bits of resolution
}


unsigned int fan_speed()
// Returns the fan speed, or 0 on error.  Minimum returned is a little
// under 1000rpm.  Anything less is error.
{
  int i, j;
  int v;
  int max_loops = 200; // Large enough to allow a low speed fan
  int loop_delay = 100; // Small enough to give good resolution
  
  v = digitalRead(FAN_PULSE);
  
  // Look for a transition A->B
  
  for (i = 0; i < max_loops; i++)
  {
    delayMicroseconds(loop_delay);
    
    if (digitalRead(FAN_PULSE) != v)
      break;
  }
  
  if (i >= max_loops)
    return 0;
    
  v = digitalRead(FAN_PULSE);
  
  // Look for next transition B->A
  
  for (i = 1; i < max_loops; i++)
  {
    delayMicroseconds(loop_delay);
    
    if (digitalRead(FAN_PULSE) != v)
      break;
  }
  
  if (i >= max_loops)
    return 0;
  
  // Look for transition A->B which
  // is one complete period
  
  for (j = 1; j < max_loops; j++)
  {
    delayMicroseconds(loop_delay);
    
    if (digitalRead(FAN_PULSE) == v)
      break;
  }
  
  if (j >= max_loops)
    return 0;
    
  i = i+j;
  
  // Magic numbers galore - return RPM at two pulses per revolution
  
  return (300000/i);
}


// -----------------------------------------------------------------------------------

void loop() {
  static float last_power = 0.0;
  static int direction = -1; // Perturb and observe direction of travel
  float voltage, current, power, cap_voltage;
  uint16_t voltage_sample, current_sample;
  uint16_t sample_tot = 0;
  static float temperature = 0.0; // Temperature
  
  delay(20);

  // Read the temperature and set fan accordingly
  
  if (millis() > timeout_temp+10000L) {
    temp_sensor.requestTemperatures();
    
    temperature = temp_sensor.getTempCByIndex(0);

    // While not necessary, the addition of a fan will help things
    // run cooler.  The fan is turned on/off based on temperature.
    // Set the fan, either ON or OFF
    
    if (temperature > FAN_ON_TEMP)  // Degrees Celsius
      digitalWrite(FAN_DRIVE, 1);
    else if (temperature < FAN_OFF_TEMP)
      digitalWrite(FAN_DRIVE, 0);
      
    timeout_temp = millis();
  }
    
  voltage_sample = analogRead(SENSE_VOLTAGE);
  voltage = (float)voltage_sample/1024.0*VREF*(R1+R2)/R2;

  // Read a few samples to improve accuracy
  
  for (int i = 0; i < 10; i++) {
    sample_tot += analogRead(SENSE_CURRENT);
    delay(1);
  }
  current_sample = sample_tot/10;
  
  // Need to work this out!!!
  // 400mV/A for ACS723-10AU
  current = (float)current_sample/1024.0*VREF;
  current -= 0.47; // Datasheet says 0.1*Vcc offset (0.5V) but I measure 0.47V
  current *= 2.69; // Datasheet says 2.5 A/V but I measure 2.69???
  
  power = voltage*current;
  
  // Output capacitor voltage
  
  cap_voltage = analogRead(SENSE_CAP); // Coincidentally Volts == sample value!
  
#ifdef DEBUG
  Serial.print("State ");
  Serial.print(state);
  Serial.print("  Voltage ");
  Serial.print(voltage);
  Serial.print("  Current ");
  Serial.print(current);
  Serial.print("  Power ");
  Serial.print(power);
  Serial.print("  PWM ");
  Serial.print(pwm);
  Serial.print("  Cap ");
  Serial.print(cap_voltage);
  Serial.print("  Temp ");
  Serial.print(t);
  Serial.print("  Fan ");
  Serial.println(fan_speed());
#endif

  // Process input from the serial port (from user/master controller)
  // Valid commands:
  // i0;      Request information from module 0
  // h0;      Tell module 0 to halt
  // r0;      Tell module 0 to reset
  // p0:100;  Tell module 0 maximum power output is 100 Watts
  
  while (Serial.available() > 0) {
    cli[cli_pos] = Serial.read();
    
    if (isspace(cli[cli_pos]))
      continue;
      
    if ((cli_pos == LINE_LENGTH-1) || (cli[cli_pos] == ';')) {
      cli[cli_pos] = '\0';
      
      if ((cli_pos > 1) && (atoi(cli+1) == id)) {
        switch(cli[0]) {
          case 'i': // Information request
            // i;id;state;pwm;voltage;current;power;cap_voltage;temperature;fan_speed
            Serial.print("i;");
            Serial.print(id);
            Serial.print(";");
            Serial.print(state);
            Serial.print(";");
            Serial.print(pwm);
            Serial.print(";");
            Serial.print(voltage);
            Serial.print(";");
            Serial.print(current);
            Serial.print(";");
            Serial.print(power);
            Serial.print(";");
            Serial.print(cap_voltage);
            Serial.print(";");
            Serial.print(temperature);
            Serial.print(";");
            Serial.println(fan_speed());
            
            // Also report error status
            
            Serial.print("e;");
            Serial.print(id);
            Serial.print(";");
            Serial.print(error_code);
            Serial.println(";");
            break;
            
          case 'h': // Halt
            // When turning off, try to dump the energy from the inductor
            // into the load rather than just 'letting go' which ends up
            // creating a voltage spike which the TVS then has to handle
            
            OCR1B = 0;
            delay(1); // 1ms is heaps of time for things to settle
            
            digitalWrite(SYS_ENABLE, 0);
            digitalWrite(SYS_OUTPUT, 0);

#ifdef DEBUG
            Serial.println("Being told to halt");
#endif

            // If transitioning into HALT state, save to NVRAM
            
            if (state != S5) {
#ifdef DEBUG
              Serial.println("Saving halt state in NVRAM");
#endif
              EEPROM.write(EEPROM_ADDRESS_HALTED, 1);
            }
            
            state = S5;
            pwm = 0;
            break;

          case 's': // Start (used when in halted state to exit halted state)
#ifdef DEBUG
            Serial.println("Being told to start");
#endif

            // Clear possible HALT state in NVRAM

            if (EEPROM.read(EEPROM_ADDRESS_HALTED))
            {
#ifdef DEBUG
              Serial.println("Clearing halt state in NVRAM");
#endif
              EEPROM.write(EEPROM_ADDRESS_HALTED, 0);
            }
            
            state = S0;
            pwm = 0;
            
            timeout_pause = millis();
            error_code = ERROR_NONE;
            break;
            
          case 'r': // Reset
            OCR1B = 0;
            delay(1); // 1ms is heaps of time for things to settle
            
            digitalWrite(SYS_ENABLE, 0);
            digitalWrite(SYS_OUTPUT, 0);

#ifdef DEBUG
            Serial.println("Being told to reset");
#endif

            if (EEPROM.read(EEPROM_ADDRESS_HALTED))
            {
              state = S5;
#ifdef DEBUG
              Serial.println("Module is halted");
#endif
            }
            else {
              state = S0;
#ifdef DEBUG
              Serial.println("Module starting");
#endif
            }
              
            pwm = 0;
            
            timeout_pause = millis();
            error_code = ERROR_NONE;
            break;

          case 'p': // Set user-defined power limit
            {
              uint16_t mp = atoi(cli+3);

#ifdef DBEUG
              Serial.print("Received command to set maximum power to ");
              Serial.print(mp);
              Serial.println(" Watts");
#endif

              if ((mp >= MIN_MAX_POWER) && (mp <= MAX_POWER)) {
                if (mp != max_power) {
#ifdef DEBUG
                  Serial.print("Setting maximum power to ");
                  Serial.print(mp);
                  Serial.println(" Watts");
#endif
                  max_power = mp;
                  EEPROM.write(EEPROM_ADDRESS_MAX_POWER, max_power>>1);
                } else {
#ifdef DEBUG
                  Serial.println("Maximum power is already set to this level");
#endif
                }
              } else {
                // Ignore
#ifdef DEBUG
                Serial.println("Specified power level is outside acceptable range!");
#endif
              }
            }
            break;
        }
      }

      cli_pos = 0;
    }
    else
      cli_pos++;
  }
  
  
  // Finite state machine; where the action happens
  
  switch (state)
  {
    case S0:
    if (millis() > timeout_pause+3000L) // Milliseconds
    {
      // This is normal
#ifdef DEBUG
      Serial.println("S0->S1 normal timeout");
#endif
      state = S1;
    }
    break;
    
    case S1:
    if (voltage > 18.0)
    {
      // This is normal
#ifdef DEBUG
      Serial.println("Enabling boost");
      Serial.println("S1->S2 due to voltage > 18V");
#endif
      digitalWrite(SYS_ENABLE, 1);
      state = S2;
      pwm = 0;
    }
    break;
    
    case S2:
    if (voltage < 15.0)
    {
      // This happens when there's not enough juice
#ifdef DEBUG
      Serial.println("S2->S0 due to voltage < 15V");
#endif
      digitalWrite(SYS_ENABLE, 0);
      timeout_pause = millis();
      pwm = 0;
      state = S0;
    }

    // We have enough voltage to connect the output of the inverter
    
    if (flag_over && (cap_voltage > 320))
    {
#ifdef DEBUG
      Serial.println("Enabling output relay");
      Serial.println("S2->S3 due to sufficient cap voltage");
#endif
      digitalWrite(SYS_OUTPUT, 1);
      state = S3;
      flag_over = false;
    }

    // If sufficient voltage increase pwm.  Eventually this will
    // result in the output cap being charged up to operational
    // voltage levels
    
    if ((pwm < 275) && (voltage > 18.0))
      pwm++;
    else if ((pwm >= 5) && (voltage < 18.0))
      pwm -= 5;
    break;
    
    case S3:
    if (voltage < 15.0)
    {
      // This happens when there is not enough juice
#ifdef DEBUG
      Serial.println("Disabling output");
      Serial.println("S2->S0 due to voltage < 15V");
#endif
      digitalWrite(SYS_ENABLE, 0);
      digitalWrite(SYS_OUTPUT, 0);
      timeout_pause = millis();
      state = S0;
      pwm = 0;
    }
    
    // There is a local power maxima low down in the power band.
    // If we leave the MPPT to it, it often gets stuck at around
    // 50V and 0.2A.  We have a bit of a hack to try and power
    // through this region.
    
    if ((voltage > 50.0) && (current < 0.5))
      direction = 1;
    else {
      //
      // MPPT
      //
      // If we are consuming less power than last time, we're going in the
      // wrong direction on the load curve.  In this case, reverse direction!
    
      if (power < last_power)
        direction *= -1;
    }
    
    // Keep within both the user-configured power limit and the hard-set system limit

    if ((power > max_power) || (power > MAX_POWER))
      direction = -1;

    // Can only increase from zero
    
    if (pwm == 0)
      direction = 1;
      
    last_power = power;
    
    if (direction < 0)
    {
      if (pwm+direction > 20) // Keep the PWM above a certain minimum (what?)
        pwm += direction;
    }
    else
    {
      if (pwm+direction < 315)
        pwm += direction;
    }
   
    break;
    
    case S4: // Fault
    // Do nothing
    break;
    
    case S5: // Halt
    // Do nothing
    break;
  }

  // Sanity check on capacitor voltage.  In practice is always at
  // least 16-17V.  If it is below this something is likely wrong!
  // Eg. sense cable disconnected usually gives a 0 reading!
  // Go into fault mode in this case.
  
  // Another situation that can lead to erroneously low readings
  // is if the 5V rail is actually higher (!), say caused by a
  // fan that pulls up on the pulse line causing the rail to go to
  // 7V!  This typically causes a cap reading as low as 5V.
  // Circuit should be fixed in later versions to prevent this
  // but the idea to disallow low cap voltages is still sound.
  
  if ((state != S4) && (cap_voltage < 12))
  {
#ifdef DEBUG
    Serial.println("Possible cap voltage sense failure!  Shutting down!");
    Serial.println("->S4 due to cap voltage measured < 12V");
#endif
    digitalWrite(SYS_ENABLE, 0);
    digitalWrite(SYS_OUTPUT, 0);
    state = S4;
    pwm = 0;
    error_code = ERROR_CAP_VOLTAGE_LOW;
  }

  // If it ever gets too high, also barf.
  
  if ((state != S4) && (cap_voltage > 370))
  {
#ifdef DEBUG
    Serial.println("Over-voltage detected!  Shutting down!");
    Serial.println("->S4 due to cap voltage > 370V");
#endif
    digitalWrite(SYS_ENABLE, 0);
    digitalWrite(SYS_OUTPUT, 0);
    state = S4;
    pwm = 0;
    error_code = ERROR_CAP_VOLTAGE_HIGH;
  }
 
  // If the output of the transformer should be shorted (!) the
  // current rises very quickly:
  //
  // State 2  Voltage 31.14  Current 0.81  Power 25.25  PWM 0  Cap 17.00  Temp 0.00  Fan 0
  // State 2  Voltage 30.75  Current 3.37  Power 103.70  PWM 1  Cap 16.00  Temp 0.00  Fan 0
  // State 2  Voltage 30.60  Current 4.55  Power 139.36  PWM 2  Cap 18.00  Temp 0.00  Fan 0
  // State 2  Voltage 30.52  Current 4.65  Power 141.82  PWM 3  Cap 16.00  Temp 0.00  Fan 0
  // State 2  Voltage 30.45  Current 4.65  Power 141.46  PWM 4  Cap 16.00  Temp 0.00  Fan 0
  // State 2  Voltage 10.51  Current 6.42  Power 67.45  PWM 5  Cap 16.00  Temp 0.00  Fan 0
  //
  // Normal reading:
  // State 2  Voltage 20.09  Current 0.17  Power 3.36  PWM 0  Cap 116.00  Temp 0.00  Fan 0
  // State 2  Voltage 20.02  Current 0.18  Power 3.61  PWM 1  Cap 118.00  Temp 0.00  Fan 0
  // State 2  Voltage 20.02  Current 0.17  Power 3.35  PWM 2  Cap 105.00  Temp 0.00  Fan 0
  // State 2  Voltage 20.32  Current 0.18  Power 3.67  PWM 3  Cap 113.00  Temp 0.00  Fan 0
  // State 2  Voltage 19.94  Current 0.18  Power 3.60  PWM 4  Cap 113.00  Temp 0.00  Fan 0

  // Try to detect this happening.
    
  if ((state == S2) && (pwm > 1) && (pwm < 5) && (current > 2.0))
  {
#ifdef DEBUG
    Serial.println("Over-current detected!  Shutting down!");
    Serial.println("->S4 due to excess current");
#endif
    digitalWrite(SYS_ENABLE, 0);
    digitalWrite(SYS_OUTPUT, 0);
    state = S4;
    pwm = 0;
    error_code = ERROR_OVER_CURRENT;
  }

  // Check for zero output from current sensor too
  // A disconnected sensor will report strange values for current, Eg.
  // State 2  Voltage 20.02  Current -1.12  Power -22.41  PWM 27  Cap 251.00  Temp 0.00  Fan 0

  if ((state != S4) && (current < -0.1))
  {
#ifdef DEBUG
    Serial.println("Malfunctioning current sensor detected!  Shutting down!");
    Serial.println("->S4 due to current sensor fault");
#endif
    digitalWrite(SYS_ENABLE, 0);
    digitalWrite(SYS_OUTPUT, 0);
    state = S4;
    pwm = 0;
    error_code = ERROR_UNDER_CURRENT;
  }

  // Check for over-temperature and shut down if things get too hot

  if ((state != S4) && (temperature > MAX_TEMPERATURE))
  {
#ifdef DEBUG
    Serial.println("Over temperature!  Shutting down!");
    Serial.println("->S4 due to over temperature");
#endif
    digitalWrite(SYS_ENABLE, 0);
    digitalWrite(SYS_OUTPUT, 0);
    state = S4;
    pwm = 0;
    error_code = ERROR_OVER_TEMP;
  }
  
  // ...Or too cold.  Actually a disconnected sensor returns -127.
  
  if ((state != S4) && (temperature < MIN_TEMPERATURE))
  {
#ifdef DEBUG
    Serial.println("Under temperature!  Shutting down!");
    Serial.println("->S4 due to under temperature");
#endif
    digitalWrite(SYS_ENABLE, 0);
    digitalWrite(SYS_OUTPUT, 0);
    state = S4;
    pwm = 0;
    error_code = ERROR_UNDER_TEMP;
  }
  
  // If we are in state 4 something bad has happened.  Indicate this
  // fact by illuminating the FAULT LED
  
  if (state == S4)
    digitalWrite(SYS_FAULT, 0); // Active low
  else
    digitalWrite(SYS_FAULT, 1);

  // Larger values of PWM give more boost.  Values as low as 0 is OK.
  // 0 will still boost, albeit only for about 100ns.
  // 50 boosts a little.  100 a bit more.
  
  // Note: MPPT for a nominal 12V panel is around 18V
  
  // For a duty cycle of 0.81, PWM needs to be 263.  Looks like ZVS
  // will take place only above this duty cycle.  Update: this value
  // has likely changed with the new transformer with the single
  // secondary winding
  
  if (pwm != OCR1B)
    OCR1B = pwm;
}
