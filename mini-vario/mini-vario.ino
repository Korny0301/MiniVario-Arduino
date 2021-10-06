/*******************************************************************
   Mini Vario with Bluetooth support

   First code by IvkoPivko/MiniVario-Arduino
   Adapted by Korny0301/MiniVario-Arduino

   Features:
    Bluetooth Modul HC-06, HC-05
    Barometer Modul MS5611 or BMP280
    any Arduino board, tested with Arduino Nano, BMP280, HC-05

 *******************************************************************/

#include <Wire.h>

// constants for defining which barometer you use, do not change
//! library: https://github.com/jarzebski/Arduino-MS5611
#define _BARO_MS5611 1
//! library: https://github.com/adafruit/Adafruit_BMP280_Library
#define _BARO_BMP280 2

// add further batteries here and in battery level calculation routine
#define BATTERY_1S 1

// used for BLUETOOTH_SERIAL
// hardware solution (default RX+TX pins), result in troubles using the serial port 
// for flashing and monitoring!
#define BLUETOOTH_SERIAL_HW 1
// software solution for serial communication, any digital pins
#define BLUETOOTH_SERIAL_SW 2

// TODO WIP
#define RISE_EVALUATION_IVKOPIVKO 1
#define RISE_EVALUATION_PAVLESKI  2

#define BT_PROTOCOL_BLUEFLY 1 // TODO not sure if this works
#define BT_PROTOCOL_LK8EX1 2 // worked for me with Flyskyhy app on iOS 15
#define BT_PROTOCOL_LXNAV 3 // not tested
#define BT_PROTOCOL_CUSTOM_BFV 4 // not tested
#define BT_PROTOCOL_TEST_VIA_SERIAL 5

// ###################################################
// ###### ADAPTION AREA: adapt your config here ######

// set here the used barometer, install required library via Arduino IDE
#define BARO _BARO_BMP280

// set your used battery for lookup table of capacity
#define USED_BATTERY BATTERY_1S

// define the used serial interface
#define BLUETOOTH_SERIAL BLUETOOTH_SERIAL_SW

// protocol to use for communication via bluetooth, depends on used app/vario
#define BT_PROTOCOL BT_PROTOCOL_LK8EX1

// TODO WIP
#define RISE_EVALUATION RISE_EVALUATION_IVKOPIVKO

// speaker and led init sequence to show that device is up
#define INTRO_SEQUENCE_TONE 0

// battery blink sequence at startup that indicates the available capacity
#define INTRO_SEQUENCE_BATT 1

// use low power mode in every cyclic iteration for optimized power consumption
#define ENTER_CYCLIC_LOW_POWER_STATE 1

#if ENTER_CYCLIC_LOW_POWER_STATE
// possible values: SLEEP_30MS, SLEEP_60MS, SLEEP_120MS, SLEEP_250MS, SLEEP_500MS, SLEEP_1S, ...
# define CYCLIC_LOW_POWER_DURATION SLEEP_120MS
// use low power cycle instead of fixed timings for cyclic operations
# define CYCLIC_TIMING_OF_LOW_POWER_WAKEUP 1
# include "LowPower.h"
#endif

#define DEBUG_NO_TONE 0

#define DEBUG_BARO_READ 0
#define DEBUG_RISE_CALCULATION 0
#define DEBUG_BATTERY_VOLTAGE 0
#define DEBUG_BT_READ 0
#define DEBUG_BT_CYCLIC_WRITE 0

#define ARRAYSIZE(x) sizeof(x)/sizeof(x[0])

#define SEALEVELPRESSURE_HPA 1013.25

// variables to adapt
// define your used pins here, use number definition, e.g. D2 = 2
// pin for additional LED that shows alive state and battery level at startup
const short PinPowerLed = 2;

// pin for external speaker/beeper/buzzer
const short PinSpeaker = 3;

// pin that defines if bluetooth should be enabled
const short PinBluetoothEnabled = 4;

// pin to measure analog voltage of attached battery
const short PinBattery_analog = A3;

#if BLUETOOTH_SERIAL == BLUETOOTH_SERIAL_SW
// pins can be defined for software solution only
const short BluetoothSerialSwPinRx = 6;
const short BluetoothSerialSwPinTx = 5;
#endif

const float V_ref = 5.17; // measured voltage, should be 5V

// I2C address for BMP280 barometer
const short BMP280_BaroI2cAddress = 0x76;

const char* BluetoothName = "Kovario";

#define BT_USE_LN_FOR_CMD_EXECUTION 0

# if RISE_EVALUATION == RISE_EVALUATION_IVKOPIVKO
#define AMOUNT_AVG_VALS 8 // Anzahl Werte fuer Mittelwert bilden
const float min_steigen = 0.20; // Minimale Steigen (Standard Wert ist 0.4m/s)
const float max_sinken = -3.50; // Maximales Sinken (Standard Wert ist - 1.1m/s)
// Filter Einstellungen!!! Hier Veraenderungen nur sehr vorsichtig vornehmen!!!
const float FehlerV = 3.000 * min_steigen; // Gewichtung fuer Vario Filter berechnen. 0.1 > FehlerV < 1.0
#elif RISE_EVALUATION == RISE_EVALUATION_PAVLESKI
#  define NUM_PRESSURES 64
#  define NUM_TOTALS 16
#endif

#if ENTER_CYCLIC_LOW_POWER_STATE && CYCLIC_TIMING_OF_LOW_POWER_WAKEUP
const long leseZeit_ms = 0;
const long leseZeitBT_ms = 0;
const long BlinkAliveDuration_ms = 0;
#else
const long leseZeit_ms = 100; // interval to read data from barometer
const long leseZeitBT_ms = 100; // cyclic interval to send current data via bluetooth
const long BlinkAliveDuration_ms = 100;
#endif

// after this amount of milliseconds the status led is going to blink for the given duration
const long CycleTimeBlinkAlive_ms = 3000;

const long CycleTimeBattery_ms = 10000;

const short PowerLevelStages = 5;

// ###################################################
// ######         END OF ADAPTION AREA          ######

#if DEBUG_NO_TONE
#define tone(x, y, z)
#endif

#if BLUETOOTH_SERIAL == BLUETOOTH_SERIAL_HW
#  define DEBUG_PRINT(x)           do { if (!BluetoothEnabled) { Serial.print(x); }} while(0)
#  define DEBUG_PRINTA(x, a)       do { if (!BluetoothEnabled) { Serial.print(x, a); }} while(0)
#  define DEBUG_PRINTLN(x)         do { if (!BluetoothEnabled) { Serial.println(x); }} while(0)
#  define DEBUG_PRINTLNA(x, a)     do { if (!BluetoothEnabled) { Serial.println(x, a); }} while(0)
#else
#  define DEBUG_PRINT(x)           do { Serial.print(x); } while(0)
#  define DEBUG_PRINTA(x, a)       do { Serial.print(x, a); } while(0)
#  define DEBUG_PRINTLN(x)         do { Serial.println(x); } while(0)
#  define DEBUG_PRINTLNA(x, a)     do { Serial.println(x, a); } while(0)
#endif

#define BLUETOOTH_PRINT(x)       do { if (BluetoothEnabled) { SerialBT.print(x); }} while(0)
#define BLUETOOTH_PRINTA(x, a)   do { if (BluetoothEnabled) { SerialBT.print(x, a); }} while(0)

#if BT_USE_LN_FOR_CMD_EXECUTION
#  define BLUETOOTH_PRINTLN(x)     do { if (BluetoothEnabled) { SerialBT.println(x); }} while(0)
#  define BLUETOOTH_PRINTLNA(x, a) do { if (BluetoothEnabled) { SerialBT.println(x, a); }} while(0)
#else
#  define BLUETOOTH_PRINTLN(x)     do { if (BluetoothEnabled) { SerialBT.print(x); }} while(0)
#  define BLUETOOTH_PRINTLNA(x, a) do { if (BluetoothEnabled) { SerialBT.print(x, a); }} while(0)
#endif

#if BARO == _BARO_MS5611
# include <MS5611.h>
MS5611 bpm;
#elif BARO == _BARO_BMP280
# include <Adafruit_Sensor.h>
# include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp; // I2C
#else
# error Defined barometer not known
#endif

#if BLUETOOTH_SERIAL == BLUETOOTH_SERIAL_HW
HardwareSerial SerialBT = Serial;
#elif BLUETOOTH_SERIAL == BLUETOOTH_SERIAL_SW
#  include <SoftwareSerial.h>
SoftwareSerial SerialBT(BluetoothSerialSwPinRx, BluetoothSerialSwPinTx);
#endif

//! error number is indicated by state LED
typedef enum error_state_ {
  err_baro_init_MS5611 = 1,
  err_baro_init_BMP280,
}error_state_e;

// global variables
long Druck, Druck0, DruckB;

bool BluetoothEnabled = false;
int XOR, c, Vbat;
float Vario, VarioR, Hoehe, AvrgV, Battery_perc, Temp;

unsigned long dZeit;

#if RISE_EVALUATION == RISE_EVALUATION_IVKOPIVKO
float kal[AMOUNT_AVG_VALS];
#elif RISE_EVALUATION == RISE_EVALUATION_PAVLESKI
uint32_t pressure[NUM_PRESSURES];
uint32_t old_total[NUM_TOTALS];
int pressure_index = 0;
int total_index = 0;
uint32_t total;
int current_tone = 0;
int beep_time = 0;
#endif

// initialization at startup
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PinPowerLed, OUTPUT);

  SetStatusLeds(LOW);

  analogReference(DEFAULT); // default equals external voltage, here 5V

  pinMode(PinBluetoothEnabled, INPUT);                 
  BluetoothEnabled = digitalRead(PinBluetoothEnabled);

  // for debug print to PC
  Serial.begin(9600);

  if (BluetoothEnabled) {
    SerialBT.begin(9600); // TODO
  }
  
  DEBUG_PRINTLN("Starting mini vario project...");
  DEBUG_PRINT("Bluetooth: ");
  DEBUG_PRINTLN(BluetoothEnabled ? "enabled" : "disabled");
  
#if BARO == _BARO_MS5611
  // Initialize MS5611 sensor!
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER

  while (!bpm.begin(MS5611_ULTRA_HIGH_RES)) {
    DEBUG_PRINTLN("Error: Could not initialize MS5611 sensor");
    ShowErrorState(err_baro_init_MS5611);
  }

#elif BARO == _BARO_BMP280
  while (!bmp.begin(BMP280_BaroI2cAddress)) {
    // Could not find a valid BMP280 sensor
    DEBUG_PRINTLN("Error: Could not initialize BMP280 sensor");
    ShowErrorState(err_baro_init_BMP280);
  }

  // Default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time
#endif

#if INTRO_SEQUENCE_TONE
  // Spielt die Start-Tonfolge.
  tone(PinSpeaker, 100, 150);
  delay(200);
  tone(PinSpeaker, 200, 150);
  delay(200);
  tone(PinSpeaker, 400, 150);
  delay(200);
  tone(PinSpeaker, 700, 150);
  delay(200);
  tone(PinSpeaker, 1100, 150);
  delay(200);
  tone(PinSpeaker, 1600, 150);
  delay(200);
#endif

#if INTRO_SEQUENCE_BATT
  AkkuVolt(); // read battery level once at startup
  int batteryStage = (Battery_perc / 100.0) * (PowerLevelStages + 1);
  DEBUG_PRINT("BatteryLevelStage = ");
  DEBUG_PRINT(batteryStage);
  DEBUG_PRINT(" / ");
  DEBUG_PRINT(PowerLevelStages);
  DEBUG_PRINT(" (");
  DEBUG_PRINTA(Battery_perc, 2);
  DEBUG_PRINTLN(")");
  BlinkLedState(batteryStage);
#endif

  // rename bluetooth device
  if (BluetoothEnabled) {
      BLUETOOTH_PRINT("AT+NAME");
      BLUETOOTH_PRINTLN(BluetoothName); // set new bluetooth name
      delay(500);

#if DEBUG_BT_READ
      BLUETOOTH_PRINTLN("AT+NAME?");
      delay(500);
      DEBUG_PRINT("BT read name: ");
      while (SerialBT.available()) {
        char c = SerialBT.read();
        DEBUG_PRINT(c);
      }
#endif

      BLUETOOTH_PRINTLN("AT+RESET");
      delay(500);
  }

#if RISE_EVALUATION == RISE_EVALUATION_PAVLESKI
  BaroAuslesen();
  uint32_t p = Druck;
  total = p*NUM_PRESSURES;
  for(int i = 0; i<NUM_PRESSURES; i++)
  {
    pressure[i] = p;
  }
  for(int i = 0; i<NUM_TOTALS; i++)
  {
    old_total[i] = total;
  }
#endif

  DEBUG_PRINTLN("Initialization done.");
}

static void handleRiseTone()
{
#if RISE_EVALUATION == RISE_EVALUATION_IVKOPIVKO
  if (Vario >= min_steigen || Vario <= max_sinken) {
    PiepserX();
  }
  else {
    noTone(PinSpeaker);
  }
#elif RISE_EVALUATION == RISE_EVALUATION_PAVLESKI
  total -= pressure[pressure_index];
  pressure[pressure_index] = Druck;
  total += pressure[pressure_index];
  int32_t rate = total - old_total[total_index];
  float frate = (float)rate;
  frate = 0.0;
  old_total[total_index] = total;
  pressure_index++;
  total_index++;
  if(pressure_index >= NUM_PRESSURES)pressure_index = 0;
  if(total_index >= NUM_TOTALS)total_index = 0;
  if(rate < -200){
    if(beep_time <5)
      tone(PinSpeaker, 500 - rate, 1000);
    else
      noTone(PinSpeaker);
  }
  else if(rate > 200)
  {
    float f = 100.0 + 40000.0 * 1.0/((float)rate);
    tone(PinSpeaker, f, 1000);
  }
  else
  {
    noTone(PinSpeaker);
  }
  beep_time++;
  if(beep_time >= 10)beep_time = 0;
  
#  if DEBUG_RISE_CALCULATION
  DEBUG_PRINT("pressure_index=");
  DEBUG_PRINT(pressure_index);
  
  DEBUG_PRINT("; pres[Pa]=");
  DEBUG_PRINT(Druck);
  
  DEBUG_PRINT("; h[m]=");
  DEBUG_PRINTA(Hoehe, 2);
  
  DEBUG_PRINT("; rate=");
  DEBUG_PRINT(rate);
  
  DEBUG_PRINT("; total=");
  DEBUG_PRINTLN(total);
#  endif
#else
#  error This rise evaluation is not known!
#endif
}

// cyclic loop
void loop()
{
  static bool start = true;
  static unsigned long lastTimeDiffBattery_ms;
  static unsigned long lastTimeBlinkAlive_ms;
  static unsigned long lastTimeDiff_ms;
  static unsigned long lastTimeBluetooth_ms;
  unsigned long diff_ms;
  unsigned long timeNow_ms = micros() / 1000;

  if (start) {
    // start blinking after waiting the very first cycle time for enough off 
    // time after power state blinking
    lastTimeBlinkAlive_ms = timeNow_ms;
  }
  
  if ((timeNow_ms - lastTimeDiffBattery_ms) >= CycleTimeBattery_ms) {
    AkkuVolt();
    lastTimeDiffBattery_ms = timeNow_ms;
  }

  if ((timeNow_ms - lastTimeBlinkAlive_ms) >= CycleTimeBlinkAlive_ms) {
    SetStatusLeds(HIGH);
    lastTimeBlinkAlive_ms = timeNow_ms;
  }
  else if ((timeNow_ms - lastTimeBlinkAlive_ms) >= BlinkAliveDuration_ms) {
    SetStatusLeds(LOW);
  }
    
  if ((diff_ms = timeNow_ms - lastTimeDiff_ms) >= leseZeit_ms) {
    BaroAuslesen();
#if RISE_EVALUATION == RISE_EVALUATION_IVKOPIVKO
    SteigenBerechnen(diff_ms);
#endif
    lastTimeDiff_ms = timeNow_ms;
  }
  
  handleRiseTone();
  
  if (BluetoothEnabled) {
    if ((timeNow_ms - lastTimeBluetooth_ms) >= leseZeitBT_ms) {
      Bluetooth();
      lastTimeBluetooth_ms = timeNow_ms;
    }
  }

#if ENTER_CYCLIC_LOW_POWER_STATE
  LowPower.idle(CYCLIC_LOW_POWER_DURATION, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                SPI_OFF, USART0_OFF, TWI_OFF);
#endif

  start = false;
}

static void SetStatusLeds(int state)
{
  digitalWrite(LED_BUILTIN, state);
  digitalWrite(PinPowerLed, state);
}

static void BlinkLedState(int amountBlinks)
{
  const int durationLow = 150;
  const int durationHigh = 350;
  
  while (amountBlinks--) {
    SetStatusLeds(HIGH);
    delay(durationHigh);
    SetStatusLeds(LOW);
    delay(durationLow);
  }
}

static void ShowErrorState(error_state_e err)
{
  const int durationWait = 1000;
  const int durationLow = 150;
  const int durationHigh = 350;

  SetStatusLeds(LOW);
  delay(durationWait);
  
  BlinkLedState(err);
}

// read all available information from external barometer sensor
static void BaroAuslesen()
{
#if BARO == _BARO_MS5611
  Temp = bpm.readTemperature();
  Druck = bpm.readPressure(true);
  Hoehe = bpm.getAltitude(Druck);
#elif BARO == _BARO_BMP280
  Temp = bmp.readTemperature();
  Druck = bmp.readPressure();
  Hoehe = bmp.readAltitude(SEALEVELPRESSURE_HPA);
#endif
#if DEBUG_BARO_READ
  DEBUG_PRINT("Baro: temp=");
  DEBUG_PRINT(Temp);
  DEBUG_PRINT(", pressure=");
  DEBUG_PRINT(Druck);
  DEBUG_PRINT(", height=");
  DEBUG_PRINTLN(Hoehe);
#endif
}

#if RISE_EVALUATION == RISE_EVALUATION_IVKOPIVKO
static void SteigenBerechnen(float timeDiff_ms)
{
  static int startCh = 0;
  if (!startCh) {
    kal[0] = Hoehe;
    startCh = 1;
  }

  // Steigwerte berechnen.
  VarioR = ((Hoehe - kal[0]) / (timeDiff_ms / 1000));

  //VarioR=0.500; // Ton Test ! In normalen Betrieb auskommentieren!  ###################
  //kal[1] = VarioR;

  kal[1] = 0.55 * VarioR + 0.45 * kal[1];

  kal[0] = Hoehe;

  // Filter fuer die Steigung anwenden.
  // > Mittelwert bilden.
  AvrgV = 0;
  for (int i = 1; i < AMOUNT_AVG_VALS; i++) {
    AvrgV += kal[i];
  }
  AvrgV = AvrgV / (AMOUNT_AVG_VALS - 1);
  AvrgV = (AvrgV + Vario) / 2;
  // < Mittelwert bilden.

  float errFactor = FehlerV > 1.0 ? 1.0 : FehlerV;
  Vario = errFactor * AvrgV + (1 - errFactor) * Vario;

  for (int i = AMOUNT_AVG_VALS - 1; i > 1; i--) {
    kal[i] = kal[i - 1];
  }

#if DEBUG_RISE_CALCULATION
  DEBUG_PRINT("BT_BTN=");
  DEBUG_PRINT(BluetoothEnabled);
  
  DEBUG_PRINT("; dZeit[ms]=");
  DEBUG_PRINTA(timeDiff_ms, 2);
  
  DEBUG_PRINT("; pres[Pa]=");
  DEBUG_PRINT(Druck);
  
  DEBUG_PRINT("; h[m]=");
  DEBUG_PRINTA(Hoehe, 2);
  
  DEBUG_PRINT("; VarioR[m/s]=");
  DEBUG_PRINTA(VarioR, 2);
  
  DEBUG_PRINT("; Vario[m/s]=");
  DEBUG_PRINTLNA(Vario, 2);
#endif                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
}
#endif

static void AkkuVolt()
{
  // typical values to convert the available voltage into capacity
  const struct {
    unsigned short percent;
    float voltage;
  } capacity2voltage[] = {
#if USED_BATTERY == BATTERY_1S // this is only for 3.7V lipos (1S)
    { 100, 4.20 }, // I measured with active and non-active system 4.08V at 100% with my battery...
    {  90, 4.11 },
    {  80, 4.02 },
    {  70, 3.95 },
    {  60, 3.87 },
    {  50, 3.84 },
    {  40, 3.80 },
    {  30, 3.77 },
    {  20, 3.73 },
    {  10, 3.69 },
    {   0, 3.27 },
#else
# error Used battery is not defined with it's capacity levels, do some internet research to find other tables
#endif
  };
  
  Vbat = analogRead(PinBattery_analog);
  const float batt_voltage = Vbat * (V_ref / 1023.00);
  
  Battery_perc = 0;
  for (int i = 0; i < ARRAYSIZE(capacity2voltage); ++i) {
    if (batt_voltage >= capacity2voltage[i].voltage) { // found sampling point of battery voltage in constant table
      Battery_perc = capacity2voltage[i].percent;
      if (i > 0) {
        // interpolate between sampling points
        Battery_perc += 10.0 * (batt_voltage - capacity2voltage[i].voltage) / (capacity2voltage[i - 1].voltage - capacity2voltage[i].voltage);
      }
      break;
    }
  }
#if DEBUG_BATTERY_VOLTAGE
  DEBUG_PRINT("Battery: analog=");
  DEBUG_PRINT(Vbat);
  DEBUG_PRINT(", batt=");
  DEBUG_PRINT(batt_voltage);
  DEBUG_PRINT("[V] = ");
  DEBUG_PRINT(Battery_perc);
  DEBUG_PRINTLN("[%]");
#endif
}

#if RISE_EVALUATION == RISE_EVALUATION_IVKOPIVKO
static void PiepserX()
{
  static unsigned long timeBeep;
  
  //Vario = 1.00; // Ton Test! In normalen Betrieb auskommentieren!

  float frequency = -0.33332 * Vario * Vario * Vario * Vario + 9.54324 * Vario * Vario * Vario - 102.64693 * Vario * Vario + 512.227 * Vario + 84.38465;

  float duration = 1.6478887 * Vario * Vario - 38.2889 * Vario + 341.275253; // Variable Pause

  frequency = int(frequency);
  duration = long(duration);

  // Wenn Steigen groesser als min_steigen
  if (Vario >= min_steigen) {
    if ((millis() - timeBeep) >= (unsigned long)(2 * duration)) {
      timeBeep = millis();
      tone(PinSpeaker , int(frequency), int(duration) );
    }
  }

  // Wenn Sinken kleiner als max_sinken
  if (Vario <= max_sinken) {
    tone(PinSpeaker, 300, 150);
    delay(125);
    tone(PinSpeaker, 200, 150);
    delay(150);
    tone(PinSpeaker, 100, 150);
    delay(175);
  }
}
#endif

// ###### Bluetooth functionality ######
// different communitation protocols possible
static void Bluetooth()
{
#if BT_PROTOCOL == BT_PROTOCOL_BLUEFLY

    /* Ausgabe im BlueFlyVario Format.     The standard BlueFlyVario outp ut mode. This sends raw
    pressure measurements in the form "PRS XXXXX\n": XXXXX is the raw (unfiltered) pressure
    measurement in hexadecimal pascals. */

    BLUETOOTH_PRINT("PRS "); // Ausgabe an der BT fuer MiniPro.
    BLUETOOTH_PRINTLNA(Druck, HEX); // BT-Serial Schnittstelle ungefiltert. Fuer MiniPro.

# if DEBUG_BT_CYCLIC_WRITE
    DEBUG_PRINT("PRS ");
    DEBUG_PRINTLNA(Druck, HEX);
# endif

    // Wenn XCSoar verwendet wird die Zeile drunter mit "//..." auskommentieren.
    //delay(leseZeitBT_ms - 22); // Wenn XCTrack benutzt wird Zeile aktiv lassen.

#elif BT_PROTOCOL == BT_PROTOCOL_LXNAV

  /* Send LXWP0 output mode for use with a range of apps:
      "$LXWP0,loger_stored (Y/N), IAS (kph), baroaltitude (m), vario (m/s),,,,,,heading of plane,
      windcourse (deg),windspeed (kph)*checksum \r\n" */
      
    String s = "LXWP0,N,,";
    s = String(s+ String(Hoehe,1) + "," + String(Vario,2) + ",,,,,,,,"  );

    // Checksum berechnen und als int ausgeben
    // wird als HEX benötigt im NMEA Datensatz
    // zwischen $ und * rechnen
    int i, XOR, c;
    XOR = 0;

    // checksum berechnen
    for (i = 0; i < s.length(); i++) {
        c = (unsigned char)s.charAt(i);
        if (c == '*') break;
        if (c!='$') XOR ^= c;
    }

    // Fuer MiniPro:
    BLUETOOTH_PRINT("$");
    BLUETOOTH_PRINT(s);
    BLUETOOTH_PRINT("*");
    BLUETOOTH_PRINTLNA(XOR,HEX);

# if DEBUG_BT_CYCLIC_WRITE
    DEBUG_PRINT("$");
    DEBUG_PRINT(s);
    DEBUG_PRINT("*");
    DEBUG_PRINTLNA(XOR,HEX);
# endif
    
#elif BT_PROTOCOL == BT_PROTOCOL_LK8EX1

  // Send $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  /*
    LK8EX1,pressure,altitude,vario,temperature,battery,*checksum

    Field 0, raw pressure in hPascal:
      hPA*100 (example for 1013.25 becomes  101325)
      no padding (987.25 becomes 98725, NOT 098725)
      If no pressure available, send 999999 (6 times 9)
      If pressure is available, field 1 altitude will be ignored

    Field 1, altitude in meters, relative to QNH 1013.25
      If raw pressure is available, this value will be IGNORED (you can set it to 99999
      but not really needed)! (if you want to use this value, set raw pressure to 999999)

    Field 2, vario in cm/s
      If vario not available, send 9999  (4 times 9) Value can also be negative

    Field 3, temperature in C , can be also negative
      If not available, send 99

    Field 4, battery voltage or charge percentage Cannot be negative
      If not available, send 999 (3 times 9)
      Voltage is sent as float value like: 0.1 1.4 2.3  11.2
      To send percentage, add 1000. Example 0% = 1000
      14% = 1014 .  Do not send float values for percentages.
    Percentage should be 0 to 100, with no decimals, added by 1000!
  */

  String s = "LK8EX1,";
  s = String(s + String(Druck, DEC) + ",99999," + String(Vario*100,2) + "," + String(Temp, 1) + "," + String(Battery_perc + 1000, 0) + ",");

  // Checksum berechnen und als int ausgeben
  // wird als HEX benötigt im NMEA Datensatz
  // zwischen $ und * rechnen
  int i, XOR, c;
  XOR = 0;
  for (i = 0; i < s.length(); i++) {
    c = (unsigned char)s.charAt(i);
    if (c == '*') break;
    if (c != '$') XOR ^= c;
  }

  BLUETOOTH_PRINT("$");
  BLUETOOTH_PRINT(s);
  BLUETOOTH_PRINT("*");
  BLUETOOTH_PRINTLNA(XOR, HEX);

# if DEBUG_BT_CYCLIC_WRITE
  DEBUG_PRINT("$");
  DEBUG_PRINT(s);
  DEBUG_PRINT("*");
  DEBUG_PRINTLNA(XOR, HEX);
# endif
  
#elif BT_PROTOCOL == BT_PROTOCOL_CUSTOM_BFV

  // Start "Custom BFV" sentence =================================================================================
  /* Custom BFV sentence: This sends a NMEA like sentence in the following format:

    "$BFV,pressure(Pa),vario(cm/s), temp(deg C), battery(%),pitotDiffPressure(pa)*checksum\r\n"

    Pressure (the filtered pressure as an unsigned integer), vario (the filtered vario as an signed integer)
    and temp(signed float) are always sent. Battery % (unsigned integer) is only sent for models which include
    a battery; otherwise "0" is sent. pitotDiffPressure (signed integer) is only sent when the hardware setting
    usePitot is enabled. */

    Temp = bpm.readTemperature(true);
    Druck = 0.250* bpm.readPressure(true) +  0.750* Druck;

    AkkuVolt();

    String s = "BFV,";
    //s = String(s + String(Druck,DEC) + "," + String(Vario*100,DEC) + "," + String(Temp,2) + ",");
    s = String(s + String(Druck,DEC) + ",," + String(Temp,2) + ",");
    s = String(s + String(Battery_perc,DEC) + "," );

    // Checksum berechnen
    // und als int ausgeben wird als HEX benötigt.
    // Im NMEA Datensatz zwischen $ und * rechnen.
    int i, XOR, c;
    XOR = 0;

    for (i = 0; i < s.length(); i++) {
        c = (unsigned char)s.charAt(i);
        if (c == '*') break;
        if (c!='$') XOR ^= c;
    }

    // Fuer MiniPro:
    BLUETOOTH_PRINT("$");
    BLUETOOTH_PRINT(s);
    BLUETOOTH_PRINT("*");
    BLUETOOTH_PRINTLN(XOR,HEX);

#elif BT_PROTOCOL == BT_PROTOCOL_TEST_VIA_SERIAL

  // Zum Testen ueber Serial-Port !!!-> nicht vergessen VarioR aus zu kommentieren.
  // Temp.[C°];Druck[Pa];Hoehe[m];dZeit[ms];VarioR[m/s];Vario[m/s];BT Taster

  DEBUG_PRINTA(Temp, 2);
  DEBUG_PRINT("; ");

  DEBUG_PRINT(Druck);
  DEBUG_PRINT("; ");

  DEBUG_PRINTA(Hoehe, 2);
  DEBUG_PRINT("; ");

  DEBUG_PRINTA(dZeit/1000, 3);
  DEBUG_PRINT("; ");

  DEBUG_PRINTA(VarioR, 2);
  DEBUG_PRINT("; ");

  DEBUG_PRINTA(Vario, 2);
  DEBUG_PRINT("; ");

  DEBUG_PRINT(BluetoothEnabled);
  DEBUG_PRINTLN(" ");

#else
#  error Bluetooth protocol not known
#endif
}
