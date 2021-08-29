/*******************************************************************
   Mini Vario with Bluetooth support

   First code by IvkoPivko/MiniVario-Arduino
   Adapted by Korny0301/MiniVario-Arduino

   Features:
    Bluetooth Modul H-06
    Barometer Modul MS5611 or BMP280
    any Arduino board, tested with Arduino Nano

 *******************************************************************/

#include <Wire.h>

// constants for defining which barometer you use, do not change
//! library: https://github.com/jarzebski/Arduino-MS5611
#define _BARO_MS5611 1
//! library: https://github.com/adafruit/Adafruit_BMP280_Library
#define _BARO_BMP280 2

// add further batteries here and in battery level calculation routine
#define BATTERY_1S 1

// ###################################################
// ###### ADAPTION AREA: adapt your config here ######

// set here the used barometer, install required library via Arduino IDE
#define BARO _BARO_BMP280

#define USED_BATTERY BATTERY_1S

// speaker and led init sequence to show that device is up
#define INTRO_SEQUENCE 1

#define DEBUG_BARO_READ 0
#define DEBUG_RISE_CALCULATION 0
#define DEBUG_BATTERY_VOLTAGE 1

#define ARRAYSIZE(x) sizeof(x)/sizeof(x[0])

#define SEALEVELPRESSURE_HPA 1013.25

// variables to adapt
// define your used pins here, use number definition, e.g. D2 = 2
const short bt_pin = 2; // Bluetooth Pin definieren. Fuer Leonardo 14. Fuer die Anderen 2
const short a_pin1 = 3; // Lautsprecher Pin definieren
const short BatV = A3; // Akku Spannung Pin definieren

const short PinPowerLed = 2; // Power LED

const float V_ref = 5.17; // measured voltage, should be 5V

// I2C address for BMP280 barometer
const short BMP280_BaroI2cAddress = 0x76;

const float min_steigen = 0.20; // Minimale Steigen (Standard Wert ist 0.4m/s)
const float max_sinken = -3.50; // Maximales Sinken (Standard Wert ist - 1.1m/s)

long leseZeit_ms = 125; // Interval zum lesen vom Baro audio Vario, Standard(min) ist 150
const long leseZeitBT = 100; // Interval zum lesen vom Baro fuer BT, Standard(min) ist 100
const long CycleTimeBattery_ms = 10000;

// Filter Einstellungen!!! Hier Veraenderungen nur sehr vorsichtig vornehmen!!!
float FehlerV = 3.000 * min_steigen; // Gewichtung fuer Vario Filter berechnen. 0.1 > FehlerV < 1.0

const short PowerLevelStages = 5;

// ###################################################
// ######         END OF ADAPTION AREA          ######

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

//! error number is indicated by state LED
typedef enum error_state_ {
  err_baro_init_MS5611 = 1,
  err_baro_init_BMP280,
}error_state_e;

// global variables
long Druck, Druck0, DruckB;

int PinBT, XOR, c, startCH = 0, Vbat;
float Vario, VarioR, Hoehe, AvrgV, Battery_perc, Temp;

unsigned long dZeit, ZeitE, ZeitS, ZeitPip;

#define AMOUNT_AVG_VALS 8 // Anzahl Werte fuer Mittelwert bilden
float kal[AMOUNT_AVG_VALS];


// initialization at startup
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PinPowerLed, OUTPUT);

  SetStatusLeds(HIGH);

  analogReference(DEFAULT); // default equals external voltage, here 5V

  leseZeit_ms = leseZeit_ms - 34;

  Serial.begin(9600);
  Serial.println("Starting mini vario project...");
  /*
    pinMode(bt_pin, INPUT);                 // Definiert den Pin für der BT Schalter.
    PinBT = digitalRead(bt_pin);            // Definiere SChalter Zustand fuer BT.
    //PinBT = 0;                            // Wenn keine BT-Modul eingebaut ist. Die obere Zwei auskommentieren.

    pinMode(7, OUTPUT);                     // Pin zum BT Versorgung.
    pinMode(8, OUTPUT);                     // Pin zum BT Versorgung.
  */

#if BARO == _BARO_MS5611
  // Initialize MS5611 sensor!
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER

  while (!bpm.begin(MS5611_ULTRA_HIGH_RES)) {
    Serial.println("Error: Could not initialize MS5611 sensor");
    ShowErrorState(err_baro_init_MS5611);
  }

#elif BARO == _BARO_BMP280
  while (!bmp.begin(BMP280_BaroI2cAddress)) {
    // Could not find a valid BMP280 sensor
    Serial.println("Error: Could not initialize BMP280 sensor");
    ShowErrorState(err_baro_init_BMP280);
  }

  // Default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time
#endif
  
  // BT umbenennen START
  //  if (PinBT == 1)
  //  {
  //    digitalWrite(7, HIGH);               // BT Versorgung einschalten.
  //    digitalWrite(8, HIGH);               // BT Versorgung einschalten.
  //    delay(1000);
  //    Serial.begin(9600);                  //fuer MiniPro
  //    //Serial1.begin(9600);                 //fuer BT - Leonardo.
  //    / On-Off | Hier zwischen // die * entfernen um die BT Name zu aendern.
  //      Serial.print("AT");
  //      delay(1500);
  //      Serial.print("AT+NAMEIvkosVario"); //BT Name vergeben
  //      delay(500);
  //      //Serial.print("AT+RESET");
  //      delay(500);//
  //    // PIN ist 1234 oder 0000 <= #################################################################################
  //  }
  //  else
  //  {
  //    digitalWrite(7, LOW);               // BT Versorgung einschalten.
  //    digitalWrite(8, LOW);               // BT Versorgung einschalten.
  //  }
  //BT umbenennen ENDE


#if INTRO_SEQUENCE
  // Spielt die Start-Tonfolge.
  tone(a_pin1, 100, 150);
  delay(200);
  tone(a_pin1, 200, 150);
  delay(200);
  tone(a_pin1, 400, 150);
  delay(200);
  tone(a_pin1, 700, 150);
  SetStatusLeds(LOW);
  delay(200);
  tone(a_pin1, 1100, 150);
  delay(200);
  tone(a_pin1, 1600, 150);
  delay(200);
  SetStatusLeds(HIGH);

  AkkuVolt(); // read battery level once at startup
  for (int i = 0; i < PowerLevelStages; ++i) {
    // TODO create blink code for power level
  }
#endif

  Serial.println("Initialization done.");

  ZeitS = micros();
}

// cyclic loop
void loop()
{
  static unsigned long lastTimeDiffBattery_us;
  static unsigned long lastTimeDiff_us;
  unsigned long diff_us;
  unsigned long timeNow_us = micros();
  
  if ((diff_us = timeNow_us - lastTimeDiffBattery_us) / 1000 >= CycleTimeBattery_ms) {
    AkkuVolt();
    lastTimeDiffBattery_us = timeNow_us;
  }
    
  //if (PinBT == 0) {
    if ((diff_us = timeNow_us - lastTimeDiff_us) / 1000 >= leseZeit_ms) {
      BaroAuslesen();
      SteigenBerechnen(diff_us);
      lastTimeDiff_us = timeNow_us;
    }
    if (Vario >= min_steigen || Vario <= max_sinken) {
      PiepserX();
    }
    else {
      noTone(a_pin1);
    }
  //}
  //else {
  //  Bluetooth();
  //}
}

static void SetStatusLeds(int state)
{
  digitalWrite(LED_BUILTIN, state);
  digitalWrite(PinPowerLed, state);
}

static void ShowErrorState(error_state_e err)
{
  const int durationWait = 1000;
  const int durationLow = 150;
  const int durationHigh = 350;

  SetStatusLeds(LOW);
  delay(durationWait);
  
  int amountBlinks = err;
  while (amountBlinks--) {
    SetStatusLeds(HIGH);
    delay(durationHigh);
    SetStatusLeds(LOW);
    delay(durationLow);
  }
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
  Serial.print("Baro: temp=");
  Serial.print(Temp);
  Serial.print(", pressure=");
  Serial.print(Druck);
  Serial.print(", height=");
  Serial.println(Hoehe);
#endif
}

static void SteigenBerechnen(float timeDiff_us)
{
  int i;

  if (startCH == 0) {
    kal[0] = Hoehe;
    startCH = 1;
  }

  // Steigwerte berechnen.
  VarioR = ((Hoehe - kal[0]) / (timeDiff_us / 1000000));

  //VarioR=0.500; // Ton Test ! In normalen Betrieb auskommentieren!  ###################
  //kal[1] = VarioR;

  kal[1] = 0.55 * VarioR + 0.45 * kal[1];

  kal[0] = Hoehe;

  // Filter fuer die Steigung anwenden.
  // > Mittelwert bilden.
  AvrgV = 0;
  i = 1;
  for (i; i < AMOUNT_AVG_VALS; i++) {
    AvrgV = AvrgV + kal[i];
  }
  AvrgV = AvrgV / float(AMOUNT_AVG_VALS - 1);
  AvrgV = (AvrgV  + Vario) / 2;
  // < Mittelwert bilden.

  if (FehlerV > 1.000) {
    FehlerV = 1.000;
  }
  Vario = FehlerV * AvrgV + (1 - FehlerV) * Vario;

  i = AMOUNT_AVG_VALS - 1;
  for (i; i > 1; i--) {
    kal[i] = kal[i - 1];
  }

#if DEBUG_RISE_CALCULATION
  Serial.print("BT_BTN=");
  Serial.print(PinBT);
  
  Serial.print("; dZeit[ms]=");
  Serial.print(float(timeDiff_us) / 1000, 2);
  
  Serial.print("; pres[Pa]=");
  Serial.print(Druck);
  
  Serial.print("; h[m]=");
  Serial.print(Hoehe, 2);
  
  Serial.print("; VarioR[m/s]=");
  Serial.print(VarioR, 2);
  
  Serial.print("; Vario[m/s]=");
  Serial.println(Vario, 2);
#endif                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
}

static void AkkuVolt()
{
  // typical values to convert the available voltage into capacity
  const struct {
    unsigned short percent;
    float voltage;
  } capacity2voltage[] = {
#if USED_BATTERY == BATTERY_1S // this is only for 3.7V lipos (1S)
    { 100, 4.20 },
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
# error Used battery is not defined with it's capacity levels
#endif
  };
  
  Vbat = analogRead(BatV);
  const float batt_voltage = Vbat * (V_ref / 1023.00);
  
  Battery_perc = 0;
  for (int i = 0; i < ARRAYSIZE(capacity2voltage); ++i) {
    if (batt_voltage >= capacity2voltage[i].voltage) {
      Battery_perc = capacity2voltage[i].percent;
      if (i > 0) {
        Battery_perc += 10.0 * (batt_voltage - capacity2voltage[i].voltage) / (capacity2voltage[i - 1].voltage - capacity2voltage[i].voltage);
      }
      break;
    }
  }
#if DEBUG_BATTERY_VOLTAGE
  Serial.print("Battery: analog=");
  Serial.print(Vbat);
  Serial.print(", batt=");
  Serial.print(batt_voltage);
  Serial.print("[V] = ");
  Serial.print(Battery_perc);
  Serial.println("[%]");
#endif
}

static void PiepserX()
{
  //Vario = 1.00; // Ton Test! In normalen Betrieb auskommentieren!

  float frequency = -0.33332 * Vario * Vario * Vario * Vario + 9.54324 * Vario * Vario * Vario - 102.64693 * Vario * Vario + 512.227 * Vario + 84.38465;

  float duration = 1.6478887 * Vario * Vario - 38.2889 * Vario + 341.275253; // Variable Pause

  frequency = int(frequency);
  duration = long(duration);

  // Wenn Steigen groesser als min_steigen
  if (Vario >= min_steigen) {
    if ((millis() - ZeitPip) >= (unsigned long)(2 * duration)) {
      ZeitPip = millis();
      tone(a_pin1 , int(frequency), int(duration) );
    }
  }

  // Wenn Sinken kleiner als max_sinken
  if (Vario <= max_sinken) {
    tone(a_pin1 , 300, 150);
    delay(125);
    tone(a_pin1 , 200, 150);
    delay(150);
    tone(a_pin1 , 100, 150);
    delay(175);
  }
}

// ###### Bluetooth functionality ######
// different communitation protocols possible
static void Bluetooth()
{
  // Start "Blue Fly Vario" sentence =============================================================================
  /* Ausgabe im BlueFlyVario Format.     The standard BlueFlyVario outp ut mode. This sends raw
    pressure measurements in the form "PRS XXXXX\n": XXXXX is the raw (unfiltered) pressure
    measurement in hexadecimal pascals. */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
    Temp = bpm.readTemperature();
    //Druck = bpm.readPressure();
    Druck = 0.250* bpm.readPressure(true) +  0.750* Druck;

    Serial.print("PRS ");               //Ausgabe an der BT fuer MiniPro.
    Serial.println( Druck, HEX);        //BT-Serial Schnittstelle ungefiltert.  Fuer MiniPro.

    //Serial1.print("PRS ");               //Ausgabe an der BT fuer Leonardo.
    //Serial1.println( Druck, HEX);        //BT-Serial Schnittstelle ungefiltert.  Fuer Leonardo.

    delay(leseZeitBT - 73);

    // Wenn XCSoar verwendet wird die Zeile drunter mit "//..." auskommentieren.
    //delay(leseZeitBT - 22); //Wenn XCTrack benutzt wird Zeile aktiv lassen.

    // Ende "BlueFlyVario" sentence =========================================================================== */

  // Start "LXNAV - LXWP0" sentence ==============================================================================
  // =============================================================================================================
  /* Send LXWP0 output mode for use with a range of apps:
      "$LXWP0,loger_stored (Y/N), IAS (kph), baroaltitude (m), vario (m/s),,,,,,heading of plane,
      windcourse (deg),windspeed (kph)*checksum \r\n" */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
      SteigenBerechnen();

      String s = "LXWP0,N,,";
      s = String(s+ String(Hoehe,1) + "," + String(Vario,2) + ",,,,,,,,"  );

    // Checksum berechnen und als int ausgeben
    // wird als HEX benötigt im NMEA Datensatz
    // zwischen $ und * rechnen
      int i, XOR, c;
      XOR = 0;

      for (i = 0; i < s.length(); i++) {
          c = (unsigned char)s.charAt(i);
          if (c == '*') break;
          if (c!='$') XOR ^= c;
      }
    // Checksum berechnen

      // Fuer MiniPro:
      Serial.print("$");
      Serial.print(s);
      Serial.print("*");
      Serial.println(XOR,HEX);

      // Fuer Leonardo:
      //Serial1.print("$");
      //Serial1.print(s);
      //Serial1.print("*");
      //Serial1.println(XOR,HEX); //

    delay(leseZeitBT - 73);

    // Ende "LXNAV - LXWP0" sentence ========================================================================== */

  // Start "LK8EX1" sentence =====================================================================================
  // =============================================================================================================
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
  // On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
  //    Temp = bpm.readTemperature(true);
  //    Druck = 0.250* bpm.readPressure(true) +  0.750* Druck;
  //SteigenBerechnen();

  String s = "LK8EX1,";
  s = String(s + String(Druck, DEC) + ",99999,9999," + String(Temp, 1) + "," + String(Battery_perc, 0) + ",");

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
  // Checksum berechnen

  // Fuer MiniPro:
  Serial.print("$");
  Serial.print(s);
  Serial.print("*");
  Serial.println(XOR, HEX);

  // Fuer Leonardo:
  //Serial1.print("$");
  //Serial1.print(s);
  //Serial1.print("*");
  //Serial1.println(XOR,HEX);

  delay(leseZeitBT - 30);
  // Ende "LK8EX1" sentence ================================================================================= */

  // =>>

  // Start "Custom BFV" sentence =================================================================================
  /* Custom BFV sentence: This sends a NMEA like sentence in the following format:

    "$BFV,pressure(Pa),vario(cm/s), temp(deg C), battery(%),pitotDiffPressure(pa)*checksum\r\n"

    Pressure (the filtered pressure as an unsigned integer), vario (the filtered vario as an signed integer)
    and temp(signed float) are always sent. Battery % (unsigned integer) is only sent for models which include
    a battery; otherwise "0" is sent. pitotDiffPressure (signed integer) is only sent when the hardware setting
    usePitot is enabled. */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.

      //SteigenBerechnen();
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
      Serial.print("$");
      Serial.print(s);
      Serial.print("*");
      Serial.println(XOR,HEX);

      // Fuer Leonardo:
      //Serial1.print("$");
      //Serial1.print(s);
      //Serial1.print("*");
      //Serial1.println(XOR,HEX); //

    delay(leseZeitBT - 24);

    // Ende "Custom BFV sentence" ============================================================================= */


  // Start Normale Daten Ausgabe =================================================================================
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.

    // Zum Testen ueber Serial-Port !!!-> nicht vergessen VarioR aus zu kommentieren.
    // Temp.[C°];Druck[Pa];Hoehe[m];dZeit[ms];VarioR[m/s];Vario[m/s];BT Taster
    // Zum Ausgabe aktivieren * zwischen // löschen.

    SteigenBerechnen();

    Serial.print(Temp, 2);
    Serial.print("; ");

    Serial.print(Druck);
    Serial.print("; ");

    Serial.print(Hoehe, 2);
    Serial.print("; ");

    Serial.print(dZeit/1000, 3);
    Serial.print("; ");

    Serial.print(VarioR, 2);
    Serial.print("; ");

    Serial.print(Vario, 2);
    Serial.print("; ");

    Serial.print(PinBT);
    Serial.println();

    delay(leseZeit_ms - 4);

    // Ende Normale Daten Ausgabe ============================================================================= */
}
