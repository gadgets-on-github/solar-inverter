/*
 Solar Inverter master controller - for controlling the individual modules
 
 Written by Martin van den Nieuwelaar, martin at gadgets.co.nz June 2015.
 Released under the Creative Commons Attribution Share-Alike 3.0 license.
 Absolutely no warranty etc. etc.
*/

#include <LiquidCrystal.h>
#include <GadgetsRf.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define VERSION "1.05"

/*   Release information

     1.03 - Improved documentation.
     
            Added support to reset individual modules that are in an E5 state.

     1.04 - Added delay at startup (before anything is done) to reduce damage
            when there isn't enough power and we reboot continually.

            Added a small delay between retries for reading the temperature sensor.
            Hopefully this will improve reliability and prevent modules being put
            into halt state.

     1.05 - Extended delay between retries.
*/

/*
 *  To do:
 *  
 *  When the temperature gets high, the maximum power of modules could be reduced
 *  instead of telling them to halt.  Though that doesn't fix the problem of
 *  writing to the NVRAM.
 */
 
//#define DEBUG

/* If the cylinder gets too hot it may explode.  If the temperature
   read is below the minimum there may be a sensor error. */

// Maximum temperature allowed in the HWC, in Celsius
#define MAX_HWC_TEMP 75.0
// Minimum temperature allowed in the HWC
#define MIN_HWC_TEMP -10.0
// Extreme temperature safety limit
#define EXTREME_HWC_TEMP 80.0

#define SYS_VIA 11
#define SYS_VIB 10

#define BUTTONS A0

// The length of the 'mains' waveform
#define MAINS_SEQUENCE_LENGTH 30

#define LINE_LENGTH 80

// Arduino IDE is broken so use #defines instead of enum
#define BUTTON_NONE 0
#define BUTTON_LEFT 1
#define BUTTON_RIGHT 2
#define BUTTON_UP 3
#define BUTTON_DOWN 4
#define BUTTON_SELECT 5

// There are various display modes
#define MODE_MIN 0
#define MODE_MAX 5

// The modes are defined here
#define MODE_DASH_MULT 0
#define MODE_DASH_SINGLE 1
#define MODE_TEMP_SINGLE 2
#define MODE_RESET 3
#define MODE_HALT 4
#define MODE_SET_POWER 5

// Which mode to start out in?
#define MODE_START MODE_DASH_MULT

// How many different modules are we potentially talking to?
#define MODULES 4

// RF protocol ID
#define PROTOCOL_ID 0

// Minimum time between module Wattage transmissions, in milliseconds
#define TX_INTERVAL 15000UL

// Pin to use for the 1-wire interface
#define ONE_WIRE_BUS 12

// Maxim DS18B20 temperature sensor
// A value of 9 gives 0.5C precision, 93.75ms sample time
#define TEMPERATURE_PRECISION 9

// Drive signals for the 50Hz inverter output
uint8_t mains_pos[] = {1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t mains_neg[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t mains_counter = 0;

// Pins for the LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Transmitter on pin D13
GadgetsRfTx tx(13, 26, 8000, 4000, 600, 600, 600, 1600, 600, 30000);

OneWire ds(ONE_WIRE_BUS);
DallasTemperature temp_sensor(&ds);
DeviceAddress hwcThermometer;

// Module ID we're presently displaying
int mod_display = 0;

struct {
  float v, a, w, c; // Volts, Amps, Watts, Cap voltage
  int pwm;
  float temp; // Degrees Celsius
  int fan; // RPM
  int state; // Finite State Machine
  int error; // Error code returned by inverter module

  unsigned long last_tx; // Timestamp when Watts last transmitted via RF
  unsigned long last_reset; // Timestamp when last automatically reset
} mod_st[MODULES];

char line[LINE_LENGTH];
uint8_t pos = 0;
unsigned long last_request = 0;
unsigned long timeout_temp = 0;
float hwc_temp;
uint16_t max_power = 400; // Default maximum power for module output

// Called 30 times every 50Hz cycle; 1500Hz or every 666us.

ISR(TIMER2_OVF_vect) {
  digitalWrite(SYS_VIA, mains_pos[mains_counter]);
  digitalWrite(SYS_VIB, mains_neg[mains_counter]);

  if (++mains_counter == MAINS_SEQUENCE_LENGTH)
    mains_counter = 0;
}


void setup() {
  lcd.begin(16, 2);
  lcd.print("Version ");
  lcd.print(VERSION);

#ifdef DEBUG
  lcd.print("-dbg");
#endif

  // We pause for a short while.  This is to slow down the rate at which
  // the Arduino can potentially reboot in an insufficient power situation.
  // Fewer reboots will hopefully mean less havoc.  Ideally we would only
  // be powered up by the electronics if there was plenty of juice, but
  // alas this isn't how things are.

  delay(2000);

  Serial.begin(9600);

  pinMode(BUTTONS, INPUT);

  digitalWrite(SYS_VIA, 0);
  pinMode(SYS_VIA, OUTPUT);

  digitalWrite(SYS_VIB, 0);
  pinMode(SYS_VIB, OUTPUT);

  noInterrupts();

  // We are going to use Timer 2, the 8 bit timer, for the '50Hz' generator
  // that runs at 30 times per cycle or 1500Hz (0.6666ms)

  TCCR2A = 0;
  TCCR2B = 0;

  // Setting WGM22, WGM21 and WGM20 gives
  // fast PWM from BOTTOM to OCR2A

  // Clock select.
  // OLD:Setting CS20, CS21, CS22 means div by 1024 (15625 ticks/s or 64us per tick)
  // Setting CS22 means div by 64 (250000 ticks/s or 4us per tick)

  TCCR2A |= _BV(WGM20) | _BV(WGM21);
  TCCR2B |= _BV(CS22) | _BV(WGM22); // div by 1024 (15.625kHz)

  // The interrupt mask register, TIMSKx
  TIMSK2 |= _BV(TOIE2); // Interrupt called on timer overflow

  // The TOP value that TCNT2 counts up to goes into OCR2A.
  // Using 167 gives an interrupt every 668us (1500Hz).
  OCR2A = 167; // This is the value of TOP

  // We don't bother setting OCR2B because we're not doing anything
  // based on it.

  interrupts();

  temp_sensor.begin();

  // Display number of sensors discovered

  lcd.setCursor(0, 1);
  lcd.print(temp_sensor.getDeviceCount(), DEC);
  lcd.print(" sensor; ");

  // Display whether in parasitic power mode

  lcd.print("P ");
  if (temp_sensor.isParasitePowerMode())
    lcd.print("ON");
  else
    lcd.print("OFF");

  delay(1000);

  if (!temp_sensor.getAddress(hwcThermometer, 0))
  {
    lcd.clear();
    lcd.print("dev 0 missing!");
    delay(2000);
  }

  temp_sensor.setResolution(hwcThermometer, TEMPERATURE_PRECISION);

  if (temp_sensor.getResolution(hwcThermometer) != TEMPERATURE_PRECISION)
  {
    lcd.clear();
    lcd.print("Precision error!");
    delay(2000);
  }

  temp_sensor.requestTemperaturesByAddress(hwcThermometer);
  hwc_temp = temp_sensor.getTempCByIndex(0);

  // Reset every connected module at start-up.  We do this because while we
  // might just be starting up, the modules may already be running (say we have
  // just been plugged in).  If the modules are running already, without us,
  // it's likely they will have gone into an error state.

  for (int i = 0; i < MODULES; i++) {
    Serial.print('r'); // Reset command
    Serial.print(i);
    Serial.println(";");
  }
}


char *next_value(char *p) {
  while ((*p != '\0') && (*p != ';'))
    p++;

  if (*p == '\0')
    return NULL;

  return ++p;
}


int parse() {
  // Returns the module ID or -1 on error

  char *p = line + 2;
  int module;

  if (strlen(line) < 3)
    return -1;

  // We have support for multiple message types:
  // i - information messages.
  // Eg. i;<id>;<state>;<pwm>;<voltage>;<current>;<power>;<cap_voltage>;<temperature>;<fan_speed>
  // e - error messages.
  // Eg. e;<id>;<code>;

  switch (line[0]) {
    case 'e':
      module = atoi(line + 2);

      if ((module < 0) || (module >= MODULES)) // Sanity check for module ID
        return -1;

      if ((p = next_value(p)) == NULL)
        return -1;

      mod_st[module].error = atoi(p);

      return module;
      break;

    case 'i':
      module = atoi(line + 2);

      if ((module < 0) || (module >= MODULES)) // Sanity check for module ID
        return -1;

      if ((p = next_value(p)) == NULL)
        return -1;

      // State
      mod_st[module].state = atoi(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // PWM
      mod_st[module].pwm = atoi(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // Voltage
      mod_st[module].v = atof(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // Current
      mod_st[module].a = atof(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // Power
      mod_st[module].w = atof(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // Cap voltage
      mod_st[module].c = atof(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // Temperature
      mod_st[module].temp = atof(p);

      if ((p = next_value(p)) == NULL)
        return -1;

      // Fan speed (RPM)
      mod_st[module].fan = atoi(p);
      return module;
      break;

    default:
      return -1;
  }

  return -1;
}


void show(int mode, int module) {
  switch (mode) {
    case MODE_DASH_MULT:
      lcd.home();
      lcd.print("0/");
      lcd.print(mod_st[0].state);
      lcd.print("/");
      lcd.print((int)mod_st[0].w);
      lcd.print("W");

      lcd.print(" 1/");
      lcd.print(mod_st[1].state);
      lcd.print("/");
      lcd.print((int)mod_st[1].w);
      lcd.print("   "); // Overwrite any old chars on screen

      lcd.setCursor(0, 1);
      lcd.print("2/");
      lcd.print(mod_st[2].state);
      lcd.print("/");
      lcd.print((int)mod_st[2].w);
      lcd.print("W");

      lcd.print(" 3/");
      lcd.print(mod_st[3].state);
      lcd.print("/");
      lcd.print((int)mod_st[3].w);
      lcd.print("   "); // Overwrite any old chars on screen
      break;

    case MODE_DASH_SINGLE:
      lcd.home();

      lcd.print(module);
      lcd.print("/");
      lcd.print(mod_st[module].state);

      lcd.print(" PWM ");
      lcd.print(mod_st[module].pwm);
      lcd.print(" ");
      lcd.print(mod_st[module].c, 0);
      lcd.print("V    ");

      lcd.setCursor(0, 1);

      lcd.print(mod_st[module].v, 1);
      lcd.print("V ");

      lcd.print(mod_st[module].a, 1);
      lcd.print("A ");

      lcd.print(mod_st[module].w, 0);
      lcd.print("W    ");
      break;

    case MODE_TEMP_SINGLE:
      lcd.home();

      lcd.print(module);
      lcd.print("/");
      lcd.print(mod_st[module].temp);
      lcd.print("C ");
      lcd.print(mod_st[module].fan);
      lcd.print("RPM   ");

      lcd.setCursor(0, 1);

      lcd.print(hwc_temp);
      lcd.print("C ");

      if (mod_st[module].error != 0) {
        lcd.print("E");
        lcd.print(mod_st[module].error);
      } else
        lcd.print("   "); // Erase anything that was there
      break;

    case MODE_RESET:
      lcd.home();

      lcd.print("Reset module ");
      lcd.print(module);
      lcd.print("?");
      break;

    case MODE_HALT:
      lcd.home();

      if (mod_st[module].state == 5) { // Halted
        lcd.print("Start module ");
      } else {
        lcd.print("Halt module ");
      }
      lcd.print(module);
      lcd.print("?");
      break;

    case MODE_SET_POWER:
      lcd.home();

      lcd.print("Set max ");
      lcd.print(module);
      lcd.print("/");
      lcd.print(max_power);
      lcd.print("W ");
      break;
  }
}


int read_button() {
  /* Measured, sample 1:
     SELECT 642    LEFT 412    DOWN 256     UP 99    RIGHT 0
     Measured, sample 2:
     SELECT 730    LEFT 512    DOWN 343     UP 144   RIGHT 0
     Old faulty unit from outside:
     SELECT 643    LEFT 413    DOWN 900+(!) UP 101   RIGHT 0
  */

  int adc_key_in = analogRead(BUTTONS);

#ifdef DEBUG
  if (adc_key_in < 1000) {
    Serial.print("Analog read is ");
    Serial.println(adc_key_in);
  }
#endif

  if (adc_key_in < 40)
    return BUTTON_RIGHT;
  else if (adc_key_in < 200)
    return BUTTON_UP;
  else if (adc_key_in < 370)
    return BUTTON_DOWN;
  else if (adc_key_in < 570)
    return BUTTON_LEFT;
  else if (adc_key_in < 900)
    return BUTTON_SELECT;
  else
    return BUTTON_NONE;

  /*
    if ((adc_key_in > 600) && (adc_key_in < 700))
    return BUTTON_SELECT;
    else if ((adc_key_in > 350) && (adc_key_in < 460))
    return BUTTON_LEFT;
    else if ((adc_key_in > 200) && (adc_key_in < 300))
    return BUTTON_DOWN;
    else if ((adc_key_in > 60) && (adc_key_in < 140))
    return BUTTON_UP;
    else if (adc_key_in < 40)
    return BUTTON_RIGHT;

    return BUTTON_NONE;
  */
}


int process_select(int mode) {
  switch (mode) {
    case MODE_RESET:
      lcd.clear();
      lcd.print("Sending reset...");
      Serial.print('r'); // Reset command - r<module>;
      Serial.print(mod_display);
      Serial.println(";");
      delay(1000);
      return MODE_DASH_SINGLE; // Change mode
      break;

    case MODE_HALT:
      lcd.clear();
      if (mod_st[mod_display].state == 5) { // Halted
        lcd.print("Sending start...");
        Serial.print('s'); // Start command - s<module>;
        Serial.print(mod_display);
        Serial.println(";");
      } else {
        lcd.print("Sending halt...");
        Serial.print('h'); // Halt command - h<module>;
        Serial.print(mod_display);
        Serial.println(";");
      }
      delay(1000);
      return MODE_DASH_SINGLE; // Change mode
      break;

    case MODE_SET_POWER: // p<module>:<max_power>;
      lcd.clear();
      lcd.print("Setting ");
      lcd.print(mod_display);
      lcd.print(" max to");
      lcd.setCursor(0, 1);
      lcd.print(max_power);
      lcd.print("W...");
      Serial.print('p');
      Serial.print(mod_display);
      Serial.print(':');
      Serial.print(max_power);
      Serial.println(";");
      delay(1000);
      return MODE_DASH_SINGLE; // Change mode
      break;

    default:
      return mode; // Stay in the same mode
  }
}


void loop() {
  // Modules ID we're presently requesting
  static int id = -1;
  static uint8_t mode = MODE_START;

  int b = read_button();

  switch (b)
  {
    case BUTTON_LEFT:
      if (mode == MODE_SET_POWER) {
        max_power -= 25;
        if (max_power < 50)
          max_power = 50;
        show(mode, mod_display);
      } else if (mod_display > 0) {
        lcd.clear();
        mod_display--;
        show(mode, mod_display);
      }
      delay(500);
      break;

    case BUTTON_RIGHT:
      if (mode == MODE_SET_POWER) {
        max_power += 25;
        if (max_power > 425)
          max_power = 425;
        show(mode, mod_display);
      } else if (mod_display < MODULES - 1) {
        lcd.clear();
        mod_display++;
        show(mode, mod_display);
      }
      delay(500);
      break;

    case BUTTON_UP:
      if (mode > MODE_MIN) {
        lcd.clear();
        mode--;
        show(mode, mod_display);
      }
      delay(500);
      break;

    case BUTTON_DOWN:
      if (mode < MODE_MAX) {
        lcd.clear();
        mode++;
        show(mode, mod_display);
      }
      delay(500);
      break;

    case BUTTON_SELECT:
      mode = process_select(mode);
      delay(2000);
      break;

    default:
      ;
  }

  if ((millis() - last_request) > 500UL) {
    if (++id >= MODULES)
      id = 0;

    Serial.print('i');
    Serial.print(id);
    Serial.println(";");
    last_request = millis();
  }

  // Check the temperature every 10 seconds

  if (millis() > timeout_temp + 10000UL) {

    // Ghastly hack!!!
    // Reading the temperature sometimes returns -127.0C.
    // Possibly due to interference or excessive capacitance in the line.

    for (int i = 0; i < 5; i++) // Try a few times before complaining
    {
      temp_sensor.requestTemperaturesByAddress(hwcThermometer);
      hwc_temp = temp_sensor.getTempCByIndex(0);

      if (hwc_temp > -125)
        break;

      delay(63); // Try not to sync to possible 100Hz power interference
    }

    if ((hwc_temp < MIN_HWC_TEMP) || (hwc_temp > MAX_HWC_TEMP)) {
      lcd.clear();

      if (hwc_temp > EXTREME_HWC_TEMP)
        lcd.print("HWC temp extreme");
      else if (hwc_temp > MAX_HWC_TEMP)
        lcd.print("HWC over temp!");
      else
        lcd.print("HWC under temp!");

      lcd.setCursor(0, 1);
      lcd.print(hwc_temp);
      lcd.print("C");

      for (int i = 0; i < MODULES; i++) {

        // Halt any modules not already halted
        
        if (mod_st[i].state != 5) {
          Serial.print('h'); // Halt command
          Serial.print(mod_display);
          Serial.println(";");
        }
      }

      // If the temperature gets really high, stop the PWM too.
      // This is the last-ditch fail-safe.  Without the PWM the
      // modules should not be able to produce any output at all.

      if (hwc_temp > EXTREME_HWC_TEMP) {
        for (int i = 0; i < MAINS_SEQUENCE_LENGTH; i++)
          mains_pos[i] = mains_neg[i] = 0;
      }
    }

    timeout_temp = millis();
  }


  // Kludge!  Reasonably regularly check to see if a module has an
  // under voltage error and try to reset it if conditions look favourable.

  for (int i = 0; i < MODULES; i++)
    if ((mod_st[i].error == 5) && (mod_st[i].v > 50.0) && (millis() > mod_st[i].last_reset + 10000UL)) {
      Serial.print('r'); // Reset command
      Serial.print(i);
      Serial.println(";");

      mod_st[i].error = 0; // Clear the error
      mod_st[i].last_reset = millis(); // Reset timer
    }

  // Process input

  while (Serial.available() > 0) {
    char c = Serial.read();

    line[pos] = c;

    if ((pos == LINE_LENGTH - 1) || (c == '\n')) {
      unsigned int mod_rx;

      line[pos] = '\0';
      mod_rx = parse();

      // If the update was for a screen we are watching, update it

      if ((mode == MODE_DASH_MULT) || (mod_rx == mod_display))
        show(mode, mod_display);

      if ((mod_rx < MODULES) &&
          (millis() - mod_st[mod_rx].last_tx) > TX_INTERVAL) {
        // Transmit Watts
        tx.setValue(0, 3, PROTOCOL_ID);
        tx.setValue(3, 4, mod_rx);
        tx.setValue(7, 3, mod_st[mod_rx].state);
        tx.setValue(10, 10, mod_st[mod_rx].w);
        tx.setValue(20, 6, tx.calculateCrc(0, 20));
        tx.send();

        mod_st[mod_rx].last_tx = millis();
      }

      pos = 0;
    } else {
      pos++;
    }
  }
}
