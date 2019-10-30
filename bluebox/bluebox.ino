
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <avr/sleep.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>


// Using Ultimate GPS shield and LSM9DS1
// read GPS NMEA sentences from software serial port and log minimal navigation sentences 
// (position, elevation, course and speed) every second and
// 9-axis accelerations to SD device log file. 

// IO: Use software SPI for SD card io and i2c for the accelerometer

// software SPI
#define CLK 13
#define MISO 12
#define MOSI 11
// hardware & software SPI
#define CS 10
// led for fatal error codes 
#define LED 13

// Devices:

// I2C for acceleromter

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
#define LIS_I2C 0x18

// GPS software serial
SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);

#ifndef ESP8266 // Sadly not on ESP8266
boolean usingInterrupt = false;
#endif

//#define CONSOLE
#define GPSECHO false

// logfile and messages
File logfile;
//File msgfile;

// Blink out an error code

void die(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(LED, HIGH);
      delay(150);
      digitalWrite(LED, LOW);
      delay(150);
    }
    for (i=errno; i<10; i++) {
      delay(300);
    }
  }
}

// One time setup

void setup() {

#ifndef ESP8266
#ifdef CONSOLE
  while (!Serial); // wait for serial console, Zero, Leonardo, etc.
#endif
#endif

#ifdef CONSOLE
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);
  Serial.println("Saiλdata BlueBox™ Research Prototype (v2.0.1) starting up...");
#endif

  // fatal error codes
  pinMode(LED, OUTPUT);

  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(CS, OUTPUT);
  
  // software SPI for SD card io
  if (!SD.begin(CS, MOSI, MISO, CLK)) {
#ifdef CONSOLE
    Serial.println("SD Card init failed: make sure properly formatted card is installed!");
#endif
    die(1);
  }

  // i2c for accelerometer
  if (! lsm.begin()) {
#ifdef CONSOLE
    Serial.println("LSM9DS1 9-Axis not detected!");
#endif
    die(3); 
  }
  
  /////////////////////////////////////////////
  /// init operating parameters for LSM9DS1 ///
  /////////////////////////////////////////////
  
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);    // 2, 4, 8 or 16 G!
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);     // 4, 8, 12, 16 Gauss
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS); // 245, 400, 2000 DPS
 
  
  // init file for log
  char filename[15];
  strcpy(filename, "BBXLOG00.TXT");
  
  // don't overwrite existing logfiles - find next available log file on volume
  for (uint8_t i = 0; i < 100; i++) {
    
    filename[6] = '0' + i/10; // tens
    filename[7] = '0' + i%10; // units
    
    if (! SD.exists(filename)) {
      break;
    }
  }

  // create global file handle for log IO
  logfile = SD.open(filename, FILE_WRITE);
  
  if(! logfile) {
#ifdef CONSOLE
    Serial.print("SD Could not open logfile: "); Serial.print(filename); Serial.println(" for writing!");
#endif
    die(5);
  }

  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
#ifndef ESP8266 // Not on ESP8266
  useInterrupt(true);
#endif
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
#ifndef ESP8266 // Not on ESP8266
ISR(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  #ifdef UDR0
      if (GPSECHO)
        if (c) UDR0 = c;
      // writing direct to UDR0 is much much faster than Serial.print
      // but only one character can be written at a time.
  #endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif // ESP8266

////////////////
/// MAINLOOP ///
////////////////

void loop() {
  
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to trace output to comsole this is a good time to do it!
#ifdef TRACE
      if (c) Serial.print(c);
#endif
  }
  
  // Read accelerometer data
  sensors_event_t accel, mag, gyro, temp; 
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  
  static char line[128];
  sprintf(line, "$PAXYZ,%d,%d,%d\r\n", accel.acceleration.x, accel.acceleration.y, accel.acceleration.z); // m/s^2
  uint8_t stringsize = strlen(line);
  logfile.write(line, stringsize);
  sprintf(line, "$PMXYZ,%d,%d,%d\r\n", mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
  stringsize = strlen(line);
  logfile.write(line, stringsize);
  sprintf(line, "$PGXYZ,%d,%d,%d\r\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
  stringsize = strlen(line);
  logfile.write(line, stringsize);
  
  delay(200); // wait so that we only log at 5Hz to save log space.
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    
    char *stringptr = GPS.lastNMEA();

    if (!GPS.parse(stringptr))  { // N.B. this also sets the newNMEAreceived() flag to false
      //msgfile.print("GPS unparsed NMEA sentence Rx:");
      //msgfile.println(stringptr);  
      return; // wait for another sentence from GPS unit
    }
    
    // Sentence parsed!
    /* 
    if (!GPS.fix) {
      msgfile.println("GPS No position fix");
      return;
    }
    */
    
    // write GPS NMEA log entry
    stringsize = strlen(stringptr);
    if (stringsize != logfile.write((uint8_t *)stringptr, stringsize)) {   // write the string to the SD file
        die(7);
    }
#ifdef CONSOLE
   Serial.println("GPS NMEA sentences logged");
#endif
    logfile.flush();
  }
  
}
