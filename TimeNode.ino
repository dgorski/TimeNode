#include <Timezone.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

//  #if !defined(PIN_WIRE_SDA) || !defined(PIN_WIRE_SCL)
// these are the default I2C pins for the ESP8266 ESP-12 module
#define I2C_SCL_PIN 5
#define I2C_SDA_PIN 4

// I2C device addresses
#define I2C_DEV_LCD_ADDR 0x3F
#define I2C_DEV_RTC_ADDR 0x68
#define I2C_DEV_RTC_EEPROM_ADDR 0x57

// pin for RTC square-wave - set for 1HZ in setup()
// used for updating of the LCD display (otherwise not needed)
#define RTC_SQW_PIN 13

// pins for GPS
#define GPS_TX_PIN 16
#define GPS_RX_PIN 14
// PPS is used for aligning the beginning of the second
// required for precise timing (otherwise will always be "late")
#define GPS_PPS_PIN 15

#define GPS_BAUD_RATE 9600

#define TZ_BUTTON_PIN 12

// global object instances
LiquidCrystal_I2C lcd(I2C_DEV_LCD_ADDR, 16, 2);  // address 0x3F, 2 lines x 16 chars

RTC_DS3231 rtc;
RTC_Millis soft_rtc;

SoftwareSerial gps_io(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;

// US Eastern Time Zone (New York, Detroit)
Timezone usEastern(
  (TimeChangeRule){"EDT", Second, Sun, Mar, 2, -240},
  (TimeChangeRule){"EST", First, Sun, Nov, 2, -300}
);

// US Central Time Zone (Chicago, Houston)
Timezone usCentral(
  (TimeChangeRule){"CDT", Second, Sun, Mar, 2, -300},
  (TimeChangeRule){"CST", First, Sun, Nov, 2, -360}
);

// US Mountain Time Zone (Denver, Salt Lake City)
Timezone usMountain(
  (TimeChangeRule){"MDT", Second, Sun, Mar, 2, -360},
  (TimeChangeRule){"MST", First, Sun, Nov, 2, -420}
);

// Arizona is US Mountain Time Zone but does not use DST
//Timezone usAZ((TimeChangeRule){"MST", First, Sun, Nov, 2, -420});

// US Pacific Time Zone (Las Vegas, Los Angeles)
Timezone usPacific(
  (TimeChangeRule){"PDT", Second, Sun, Mar, 2, -420},
  (TimeChangeRule){"PST", First, Sun, Nov, 2, -480}
);

Timezone UTC((TimeChangeRule){"UTC", Second, Sun, Mar, 2, 0});

Timezone *currentTimeZone = &usEastern; // default to US/Eastern
char currentTimeZoneIndex = 0;
char savedTimeZoneIndex = 0;

TimeSpan oneSecond = TimeSpan(1);

DateTime lastGpsTime_dt;
unsigned long lastGpsTime_ms;
char messagesSincePPS = 0;

/** ISR for RTC_SQW_PIN */
volatile unsigned long newRtcSecond = 0;
void rtcInterruptHandler() {
  newRtcSecond = millis();
}

/** ISR for GPS_PPS_PIN */
volatile unsigned long newGpsSecond = 0;
void gpsInterruptHandler() {
  newGpsSecond = millis();
}

/**
 * Fetches the latest time from the GPS library. Is called immediately
 * after a GPS message is received, and thus should contain the
 * latest/current date+time from the GPS receiver.  This is then used
 * to set the software reference clock
 */
void getFreshTimeStamp() {
  static boolean DEBUG_TIMESTAMP = false;

  if(DEBUG_TIMESTAMP)
    Serial.printf("\n{+TS} (%ld)\n", millis());

  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;

  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if(age == TinyGPS::GPS_INVALID_AGE) {
    if(DEBUG_TIMESTAMP)
      Serial.print("   GPS ::   date not valid: GPS_INVALID_AGE\n");
  } else {
    if(DEBUG_TIMESTAMP)
      Serial.printf("   GPS ::   %02d/%02d/%02d   %02d:%02d:%02d  (age %ld)\n", 
        year, month, day, hour, minute, second, age);

    // these two are globals
    lastGpsTime_dt = DateTime(year, month, day, hour, minute, second);
    lastGpsTime_ms = millis();

  }

  if(DEBUG_TIMESTAMP) {
    DateTime ts = rtc.now();
    Serial.printf("   RTC ::   %02d/%02d/%02d   %02d:%02d:%02d\n", ts.year(), ts.month(), ts.day(), ts.hour(), ts.minute(), ts.second());

    ts = soft_rtc.now();
    Serial.printf("   SYS ::   %02d/%02d/%02d   %02d:%02d:%02d\n", ts.year(), ts.month(), ts.day(), ts.hour(), ts.minute(), ts.second());

    Serial.println("{-TS}");
  }
}

/**
 * write the current (soft) RTC date/time to the LCD display. called via the 1HZ interrupt from
 * the RTC
 */
void updateLcdDisplay() {
  static char monthsOfTheYear[13][4] = { "", "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec" };
  static char daysOfTheWeek[7][4] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };
  static boolean showIcon = false;

  // if we have a current GPS timestamp, show the icon
  showIcon = ((millis() - lastGpsTime_ms) < 1000);

  TimeChangeRule *tcr;
  time_t ts_l = currentTimeZone->toLocal((time_t)soft_rtc.now().unixtime(), &tcr);
  DateTime ts = DateTime(ts_l);

  char lcd_line_buf[17];

  lcd.home();
  lcd.setCursor(0, 0);
  sprintf(lcd_line_buf, "%s %s %2d, %02d", daysOfTheWeek[ts.dayOfTheWeek()], monthsOfTheYear[ts.month()], ts.day(), ts.year());
  lcd.print(lcd_line_buf);
  lcd.setCursor(0, 1);
  sprintf(lcd_line_buf, "%02d:%02d:%02d %-6s%s", ts.hour(), ts.minute(), ts.second(), tcr->abbrev, showIcon ? "\a" : " ");
  lcd.print(lcd_line_buf);

}

/** read bytes from the GPS unit and feed them to the library */
void processGpsIo() {
  boolean completeMessage = false;
  while(gps_io.available()) {
    char c = gps_io.read();
    //Serial.print(c);
    if(completeMessage = gps.encode(c)) {
      break; // encode() found a complete frame
    }
  }
  if(completeMessage) {
    if(messagesSincePPS == 0) // only the first message (GPRMC)
      getFreshTimeStamp();    // after PPS has date _and_ time
    messagesSincePPS++;
  }
}

/** read the first byte from the EEPROM on the RTC module*/
byte getEepromData() {
  static int address = 0; // which position to read data from

  Wire.beginTransmission(I2C_DEV_RTC_EEPROM_ADDR);
  Wire.write((int)(address >> 8)); // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  Wire.endTransmission();

  yield();
  delay(100);

  Wire.requestFrom(I2C_DEV_RTC_EEPROM_ADDR, 1);
  while(!Wire.available()) {
    yield();
    delay(50);
  }

  byte val = Wire.read();

  return(val);
}

/** write to the first byte of the EEPROM on the RTC module*/
void setEepromData(const byte data) {
  static int address = 0; // which position to store data in
  Wire.beginTransmission(I2C_DEV_RTC_EEPROM_ADDR);
  Wire.write((int)(address >> 8)); // MSB
  Wire.write((int)(address & 0xFF)); // LSB
  delay(50);
  Wire.write(data);
  delay(50);
  Wire.endTransmission();
  
  yield();
}

/** store TZ index in EEPROM so it persists accross power-cycles */
void saveTimeZoneSelection() {
  if(currentTimeZoneIndex != savedTimeZoneIndex) {
    setEepromData(currentTimeZoneIndex);
    savedTimeZoneIndex = currentTimeZoneIndex;
  }
}

/** retreive timezone index from non-volitile memory, set runtime timezone accordingly */
void restoreTimeZoneSelection() {
  savedTimeZoneIndex = getEepromData();
  currentTimeZoneIndex = savedTimeZoneIndex;
  setTimeZone();
}

/** called on button press */
void setTimeZone() {
  if(currentTimeZoneIndex > 4) currentTimeZoneIndex = 0;
  switch(currentTimeZoneIndex) {
    case 0:
      currentTimeZone = &usEastern;
      break;
    case 1:
      currentTimeZone = &usCentral;
      break;
    case 2:
      currentTimeZone = &usMountain;
      break;
    case 3:
      currentTimeZone = &usPacific;
      break;
    case 4:
      currentTimeZone = &UTC;
      break;
  }
}

/** process a button press */
void processButton() {
  static unsigned long debounce = 0;
  static boolean processed = false;
  
  if(digitalRead(TZ_BUTTON_PIN) == LOW) { // using INPUT_PULLUP, LOW means pressed
    if(debounce == 0) { // new, store timestamp
      debounce = millis();
    } else {
      if((!processed) && (millis() - debounce > 75)) { // button press
        ++currentTimeZoneIndex;
        setTimeZone();
        processed = true;
        return;
      }
    }    
  } else {
    debounce = 0;
    processed = false;
  }
}

/** 1HZ interrupt from RTC */
void processRtcInterrupt() {
  if(newRtcSecond > 0) {
/*
    unsigned long thenMilli = newRtcSecond; // sprintf() gets zero from the volitile
    Serial.printf("\n--> 1HZ <-- (%ld / %ld)\n", thenMilli, millis());
*/
    updateLcdDisplay();
    saveTimeZoneSelection();
    newRtcSecond = 0;
  }
}

/** 1PPS interrupt from GPS */
void processGpsInterrupt() {
  messagesSincePPS = 0;

  if(newGpsSecond > 0) {
/*
    unsigned long nowMilli = millis();
    unsigned long thenMilli = newGpsSecond; // sprintf() sees zero in the volitile, copy
    Serial.printf("\n--> PPS <-- (%ld / %ld)\n", thenMilli, nowMilli);
*/
    newGpsSecond = 0; // reset
    /* If the last GPS timestamp was received less than 1 second ago
     * then it is accurate for the last second.
     * Add one second and it should now be the timestamp for the second
     * which just started, as signalled by the GPS PPS signal.  The next
     * GPS mesasge will arrive with a new timestamp in ~ 150ms at 9600bps
     * so we do this instead of doing delay(~850ms) before soft_rtc.adjust()
     */
    if(millis() - lastGpsTime_ms < 1000) {
      soft_rtc.adjust(lastGpsTime_dt + oneSecond);

      // TODO: store timestamp of last RTC update
      if(lastGpsTime_dt.minute() % 30 == 0 && lastGpsTime_dt.second() == 0) { // update RTC every 30 minutes
        rtc.adjust(soft_rtc.now()); // TODO: save and check how recently the soft_rtc was updated
      }
    }

  }
}

/**
 * adapted from http://www.forward.com.au/pfod/ArduinoProgramming/I2C_ClearBus/index.html
 * battery-backed I2C devices cause the bus to hang at restart if that device was in 
 * mid-transaction when power to the micro was lost.
 * note: this assumes default Wire pins, same way the Wire library does it
 * note: this version was only tested on ESP8266... 
 */
int clearI2C() {

#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(PIN_WIRE_SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(PIN_WIRE_SCL, INPUT_PULLUP);

  boolean SCL_LOW = (digitalRead(PIN_WIRE_SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { // If it is held low Arduno cannot become the I2C master.
    return 1;    // I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(PIN_WIRE_SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(PIN_WIRE_SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(PIN_WIRE_SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(PIN_WIRE_SCL, INPUT); // release SCL LOW
    pinMode(PIN_WIRE_SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(PIN_WIRE_SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(PIN_WIRE_SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(PIN_WIRE_SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(PIN_WIRE_SDA, INPUT); // remove pullup.
  pinMode(PIN_WIRE_SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(PIN_WIRE_SDA, INPUT); // remove output low
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(PIN_WIRE_SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(PIN_WIRE_SCL, INPUT);
  return 0; // all ok
}

void setup() {
  // for debugging. don't use RX; make that GPIO available if needed
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);

  Serial.println("\n\nStartup");

  //Serial.println("Starting Wifi");
  //wifi_setup();

  Serial.println("Clearing I2C bus");
  int i2c_state = clearI2C();
  if(i2c_state) {
    Serial.println("ERROR: Cannot clear/reset I2C bus.  Resetting.");
    ESP.reset();
  }

  Serial.println("Starting Wire");
  Wire.begin();
  delay(500); // let the I2C bus settle

  Serial.println("Setting up RTC");
  pinMode(RTC_SQW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_SQW_PIN), rtcInterruptHandler, RISING);
  rtc.writeSqwPinMode(DS3231_SquareWave1Hz); //rtc.writeSqwPinMode(DS3231_OFF);
  soft_rtc.adjust(rtc.now()); // set the software driver from the stored time
  restoreTimeZoneSelection(); // read the TZ index from EEPROM

  Serial.println("Starting LCD");
  lcd.begin(16, 2, 0); // initialize the lcd without calling Wire.begin() again
  lcd.backlight();

  byte sat[] =  { 0x00, 0x02, 0x0A, 0x04, 0x1A, 0x02, 0x07, 0x07 };
  lcd.createChar(7, sat); // "\a"

  lcd.home();
  lcd.setCursor(0, 0);
  lcd.print("Starting up...  ");
  lcd.setCursor(0, 1);
  lcd.print("                ");

  Serial.println("Starting GPS serial interface");
  attachInterrupt(digitalPinToInterrupt(GPS_PPS_PIN), gpsInterruptHandler, RISING);

  gps_io.begin(GPS_BAUD_RATE);

  pinMode(TZ_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Setup complete.");
}

void loop() {
  processGpsIo();

  processRtcInterrupt();

  processGpsInterrupt();

  processButton();

  //server.handleClient(); // web server
}
