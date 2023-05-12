/*
Firmware for underwater load cell logger - specific to the NEFSC version.

Original Author: Bill DeVoe, Marine Resource Scientist III, Maine Department of Marine Resources
Contact: william.devoe@maine.gov // bdevoe@gmail.com
Original Creation Date: 12/18/2019

License:  "THE BEER-WARE LICENSE" (Revision 42):
Bill DeVoe wrote this file.  As long as you retain this notice you
can do whatever you want with this stuff. If we meet some day, and you think
this stuff is worth it, you can buy me a beer in return.

Version History:
1.0: 5/21/2020 - Stable
2.0: 5/27/2020 - Bluefruit BLE supported
3.0: 7/17/2020 - Ported to Adafruit Feather M0 Express
3.1: 10/21/2020 - Simplified for production, no bluetooth
3.2: 11/19/2020 - Added option for monitoring LED, indicates logger recording, 
                  low battery, and if trip value has been reached.
  Fixed bug where calibration values were not being written to config file.
4.0: 12/27/2022 - Updated by M. Martini for NEFSC version of the load cell logger.  
                  Changed GPIO pins for LED control, using a common anode RGB LED
                  thus, pull down turns on the LED, so LED set values are inverted
4.1: 2/6/2023 - Updated, M. Martini to include real language for colors, and to make 
                  colors easier to change, as well as show version number in header.
4.2: 3/8/2023 - Updated, M. Martini, to return the weight entered, as the value might
                not always be correctly received depending on what serial terminal
                is used.  Increase the number of values averaged for a calibration reading.
4.3: 3/30/2023 - Updated M. Martini provide means to get gain information to user in a meaningful way

Description:
Logs values from a SparkFun Qwiic Scale NAU7802 to a SD card and specified intervals. 
Also allows for calibration of the load cell over a serial interface. Intended for 
use on a Adafruit Feather M0 Express with an Adalogger shield, but should work on other
Arduino boards with similar SD/RTC as Adalogger.

*/

#include "SD.h" // SD card
#include <Wire.h> // I2C
#include "RTClib.h" // Real time clock
#include "SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_NAU7802

// ***********************************************************************
// * MACROS
// ***********************************************************************
#define VERSION_MAJOR 4
#define VERSION_MINOR 2

// Wait for serial input in setup()?? 0 or 1
#define WAIT_TO_START    0

// Baud rate for serial port (over USB cable)
#define BAUD_RATE 9600

// ----- Default settings if not present in config file -----
// Default for echo to serial
#define DEFAULT_ECHO 1 
// Interval in milliseconds between samples
// was 250
#define DEFAULT_LOG_INTERVAL 1000 
// How many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time.
// Set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to              
// the last 10 reads if power is lost but it uses less power and is much faster!
#define DEFAULT_SYNC_INTERVAL 10000
// Default load cell calibrations
#define DEFAULT_CAL_FACTOR 0
#define DEFAULT_ZERO_OFFSET 1000L
// Default trip value in LBF
#define DEFAULT_TRIP_VALUE 1700

// Size of serial input
#define SERIAL_SIZE 15

// Number of weights to average when calibrating
// was 4
#define AVG_SIZE 20

// The digital pin to light when an error occurs. Pin 17 is built-in RX LED, usually used when receiving serial data.
// The built in LED on pin 13 cannot be used since pin 13 is being used for the clock signal for the SD card.
#define ERROR_LED 17 

// Pins for an external RGB LED for monitoring load cell status
#define STATUS_RED 11
#define STATUS_GREEN 6
#define STATUS_BLUE 5

// Analog input to measure the battery voltage. On the Feather M0 Express pin A7 (GPIO #9) is connected to a voltage divider
// such that the LiPo voltage can be measured. But GPIO #9 has already been set to be GREEN on the RGB LED. (#10, the only
// remaining PWM pin, cannot be used as it is the Card Select pin for the Adalogger). Thus it becomes necessary to save the RGB
// state prior to reading the analog voltage on this pin, and then toggle the pin state back to output after checking the voltage.
// TODO - the previous version set things this way and in the battery drain test, battery was declared low (blue LED) 
//        half way into its useful life.
#define VBATPIN A7

// Low battery voltage
#define LOW_BATTERY_VOLTAGE 3.5


// ***********************************************************************
// * GLOBALS
// ***********************************************************************
// TODO: these should probably be rolled into a singleton class/OOP structure

// due to the common anode LED used for the NEFSC logger, pull down lights the LED
// so the colors are obtained as follows
// blue 255, 255, 0
int blue[] = {255,255,0};
// green 255, 0, 255
int green[] = {255,0,255};
// red 0, 255, 255
int red[] = {0,255,255};
// magenta 0, 255, 0
int magenta[] = {0,255,0};
// yellow 10, 10, 255
int yellow[] = {10,10,255};
// orange 0, 108, 255
int orange[] = {0,108,255};
// all off 255,255,255
int all_off[] = {255,255,255};

// the gain values for the gain settings NAU7802_GAIN_xxx
// note that 0b000=0 maps to a gain of 1 and 0b111=7 maps to a gain of 128 
// defined here so they can be displayed in a meaningful way later
int gain_value_table[] = {1,2,4,8,16,32,64,128};

bool settingsDetected = false; // Used to prompt user to calibrate their scale
bool fm; // File manager is active/inactive
char input;
char serial_data[SERIAL_SIZE];

// Create an array to take average of weights. This helps smooth out jitter.
float avgWeights[AVG_SIZE];
byte avgWeightSpot = 0;

// Variables for raw and calibrated load, max load encountered. 
// Max load thus resets each power cycle
long raw_load; 
float load;
float max_load = 0;

float cal_factor; // Value used to convert the load cell reading to lbs or kg
float zero_offset; // Zero value that is found when scale is tared
 
// Pin for the the SD card select line
const int chip_select = 10;

// Globals for settings. These are read/written to CONFIG file
bool echo;
int log_interval;
int sync_interval;
int trip_value;
int gain_setting = 0;

// Time the last log was saved
uint32_t log_time = 0;
// Time the card was last synced
uint32_t sync_time = 0; 

// Array of RGB state. Pin 9 is shared with the voltage divider so have to restore led state after battery check
int rgb_state[] = {255, 255, 255};  // common anode LED

// Create instance of the Real Time Clock class
RTC_PCF8523 rtc;
// Create instance of the NAU7802 class for the load cell
NAU7802 load_cell; 

// Global now
DateTime now;

// If using the web IDE and errors are generated "does not name type" after instantiating these classes - this means the web IDE is referencing the wrong library. 
// Load the library manually as per: https://forum.arduino.cc/index.php?topic=586330.0

// Output log filename and object
char filename[12];
File logfile;

// Battery tracking variables
float measuredvbat;

// ***********************************************************************
// * SETUP
// ***********************************************************************
void setup(void) {
  Serial.begin(BAUD_RATE);  
  Serial.println();
  
  #if WAIT_TO_START
    Serial.println(F("Type any character to start"));
    while (!Serial.available()) {
      Serial.println(F("Still waiting..."));
      delay(1000);
    }
  #endif // WAIT_TO_START
  
  // Set pin mode of error led
  pinMode(ERROR_LED, OUTPUT);

  pinMode(STATUS_RED, OUTPUT);
  pinMode(STATUS_GREEN, OUTPUT);
  pinMode(STATUS_BLUE, OUTPUT);
  
  // Splash screen
  Serial.println(F("----------------------------------------"));
  Serial.println(F("      Lobster Endline Tension Meter"));
  Serial.println();
  Serial.println(F("Created by Bill DeVoe, MaineDMR"));
  Serial.println(F("For questions, email william.devoe@maine.gov"));
  Serial.println(F("Updated for NOAA NEFSC by M. Martini"));
  Serial.println(F("For questions, email marinna.martini@noaa.gov"));
  Serial.print(F("Version "));
  Serial.print(VERSION_MAJOR);
  Serial.print(F("."));
  Serial.println(VERSION_MINOR);
  Serial.println(F("----------------------------------------"));
  
  // Initialize the SD card
  Serial.println(F("Init SD card"));
  // Make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // See if the card is present and can be initialized:
  if (!SD.begin(chip_select)) {
    error(F("Card"));
  }
  Serial.println(F("SD card OK"));
  Serial.println();
  
  // Set up RTC
  Wire.begin();  
  if (!rtc.begin()) {
    error(F("RTC"));
  }

  if (! rtc.initialized()) {
    Serial.println(F("Setting RTC"));
    // Following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println(F("RTC OK"));
  Serial.println();
  
  // Set up load cell
  if (load_cell.begin() == false) {
      error(F("LC"));
  }
  Serial.println(F("LC OK"));
  
  // Increase to max sample rate
  load_cell.setSampleRate(NAU7802_SPS_320); 
  // Turn down load cell gain since we are using a large capacity cell
  // Gains of 1, 2, 4, 8, 16, 32, 64, and 128 are available. This can be adjusted
  // depending on the capacity of the cell.
  // Bill DeVoe's original gain setting was 16
  // for bench testing with nearly no load (say, 3kg=6.61 lb), try very high gain
  //gain_setting = NAU7802_GAIN_128;  
  gain_setting = NAU7802_GAIN_16;  // for real world
  load_cell.setGain(gain_setting);
  // Re-cal analog front end when we change gain, sample rate, or channel 
  load_cell.calibrateAFE();

  // Load system settings from file
  readSystemSettings();
  
  // Retrieve load cell calibration settings
  getCalibration();
  
  if (settingsDetected == false) {
      Serial.println(F("LC !cal"));
  }
  
  // Create new file based on current date and increment - 19122000.csv, 19122001.csv, 19122100.csv, etc
  now = rtc.now();
  sprintf(filename, "%02d%02d%02d00.CSV", now.year() % 1000, now.month(), now.day());
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // Only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // Leave the loop!
    } // End if SD exists
  } // End for loop
  
  if (!logfile) {
    error(F("logfile"));
  }
  
  Serial.print(F("Logging to: "));
  Serial.print(filename);
  Serial.print(F(" at "));
  Serial.print(log_interval);
  Serial.println(F("ms interval."));
  Serial.println();
  
  Serial.println();
  Serial.println(F("Type the following menu commands at any time:\n l - Change logging interval\n s - Change card sync interval\n e - Toggle echo to serial\n z - Get current real-time clock time\n d - Set real-time clock time\n c - Calibrate load cell with known weight\n m - Manually calibrate load cell with known values\n v - Retrieve load cell calibration values \n t - Tare the load cell\n f - Enter the file manager."));
  Serial.println(F("Type menu CMD any time."));
  Serial.println();
  
  // Write the file header 
  logfile.println("millis,time,raw_load,load");    
  if (echo) {
    Serial.println(F("millis,time,raw_load,load"));
  }
  if (!logfile) {
    error(F("log file"));
  }

  // Set RGB to green for setup OK
  setRGB(green, 3);

  // If you want to set the aref to something other than 5v
  //analogReference(EXTERNAL);
  
} // End setup

// ***********************************************************************
// * LOOOOOOOOOOOOOOOOOOOOOOOOOOP
// ***********************************************************************
void loop(void) {
  // Check for incoming serial data in the serial buffer
  if (Serial.available() > 0) {
    input = Serial.read();
    // Switch on incoming byte
    switch(input) {
      // Toggle echo to serial
      case 'e': case 'E':
        echo = !echo;
        Serial.println();
        if (echo) {
          Serial.println(F("EOS ON"));
        } else {
          Serial.println(F("EOS OFF"));
        }
        Serial.println();
        // Commit these values to SD config.txt
        saveSystemSettings();
        break;
      // Set logging interval
      case 'l': case 'L':
        setLogInterval();
        break;
      // Set sync interval
      case 's': case 'S':
        setSyncInterval();
        break;
      // Get zulu time
      case 'z': case 'Z':
        Serial.println(getUTC());
        break;
      // Set RTC
      case 'd': case 'D':
        setRTC();
        break;
      // Tare the load cell
      case 't': case 'T':
        load_cell.calculateZeroOffset();
        saveSystemSettings();
        Serial.println();
        Serial.println(F("LC zeroed."));
        Serial.println();
        break;
      // Calibrate the scale
      case 'c': case 'C':
        calibrateScale();
        break;
      // Output calibration settings
      case 'v': case 'V':
        getCalibration();
        break;
      // Manual calibration
      case 'm': case 'M':
        manualCalibration();
        break;
      // Enter file manager
      case 'f': case 'F':
        fileManager();
        break;
      // Invalid character entered
      default:
        Serial.println(F("Invalid command "));
        Serial.println(input);
    } // End switch
  } // End if serial available
  // Clear anything in RX buffer
  while (Serial.available()) Serial.read();
  // If the log interval has not yet elapsed, skip the rest of the loop
  if ((millis() - log_time) < log_interval) return;
  
  // Log milliseconds since starting
  log_time = millis();
  logfile.print(log_time);
  logfile.print(",");    
  if (echo) {
    Serial.print(log_time);
    Serial.print(F(","));
  }
 
  // Log time
  logfile.print(getUTC());
  if (echo) {
    Serial.print(getUTC());
  }

  // Read loadcell value and write this to line
  if (load_cell.available() == true) {
    raw_load = load_cell.getReading();
    load = load_cell.getWeight();
  } else {
    raw_load = 99999;
    load = 99999;
  }
  
  // Write load cell value to log
  logfile.print(",");
  logfile.print(raw_load);
  logfile.print(", ");
  logfile.println(load); // println ends current line in file
  if (echo) {
    Serial.print(F(","));
    Serial.print(raw_load);
    Serial.print(F(","));
    Serial.println(load);
  }

  // Check if load is greater than max_load and if so save load as max_load. Then set RGB LED.
  if (load > max_load) {
    max_load = load;
    // Set RGB LED 
    if (max_load/trip_value > 1.0) {
      setRGB(red, 3); // Red, trip value has been reached.
    } else if (max_load/trip_value > 0.75) {
      setRGB(orange, 3);  // Orange, 75% of trip value has been reached. 
    } else if (max_load/trip_value > 0.5) {
      setRGB(yellow, 3); // Yellow, 50% of trip value has been reached.
    }
  }
  
  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time; but if power is cut to board, all data since last sync is lost.
  if ((millis() - sync_time) < sync_interval) return; // Skips the rest of the loop function if not syncing
  sync_time = millis();  
  // Sync data to the card & update FAT
  if (echo) {
    Serial.println();
    Serial.println(F("Writing to SD card."));
    Serial.println();
  }
  logfile.flush();

  // Check the battery level after syncing
  measuredvbat = analogRead(VBATPIN);
  Serial.print(F("RGB pin value: ")); Serial.print(measuredvbat);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print(F("= VBat: ")); Serial.println(measuredvbat);
  // If battery voltage is below defined low battery value, set RGB light as blue
  // this overrides other states
  if (measuredvbat < LOW_BATTERY_VOLTAGE) {
    setRGB(blue, 3);
  } else {
    setRGB(rgb_state, 3);
  }
  Serial.print(F("RGB is: "));
  Serial.println(rgb_color_string(rgb_state, 3));
  
} // End loop


// ***********************************************************************
// * LOGGER/LOAD CELL FUNCTIONS
// ***********************************************************************
// TODO put these into class with globals

// Sets the RGB status LED
// void setRGB(int red_light_value, int green_light_value, int blue_light_value) {
void setRGB(int rgb_values[], int sizeOfArray) {
  int red_light_value = rgb_values[0];
  int green_light_value = rgb_values[1];
  int blue_light_value = rgb_values[2];
  analogWrite(STATUS_RED, red_light_value);
  analogWrite(STATUS_GREEN, green_light_value);
  analogWrite(STATUS_BLUE, blue_light_value);
  // Save RGB state
  rgb_state[0] = red_light_value; rgb_state[1] = green_light_value; rgb_state[2] = blue_light_value;
  Serial.print(F("RGB changed to: "));
  Serial.print(rgb_color_string(rgb_state, 3));
  Serial.print(" ");
  Serial.print(red_light_value);
  Serial.print(" ");
  Serial.print(green_light_value);
  Serial.print(" ");
  Serial.println(blue_light_value);
}

// determines rgb color intended from declared values
const char* rgb_color_string(int rgb_values[], int sizeOfArray) {
// due to the common anode LED used for the NEFSC logger, pull down lights the LED
// so the colors are obtained as follows
  if ((rgb_values[0] == blue[0]) && (rgb_values[1] == blue[1]) && (rgb_values[2] == blue[2])) {
    return("blue");  // blue 255, 255, 0
  } else if ((rgb_values[0] == green[0]) && (rgb_values[1] == green[1]) && (rgb_values[2] == green[2])) {
    return("green");  // green 255, 0, 255
  } else if ((rgb_values[0] == red[0]) && (rgb_values[1] == red[1]) && (rgb_values[2] == red[2])) {
    return("red");  // red 0, 255, 255
  } else if ((rgb_values[0] == magenta[0]) && (rgb_values[1] == magenta[1]) && (rgb_values[2] == magenta[2])) {
    return("magenta");  // magenta 0, 255, 0
  } else if ((rgb_values[0] == yellow[0]) && (rgb_values[1] == yellow[1]) && (rgb_values[2] == yellow[2])) {
    return("yellow");  // yellow 10, 10, 255
  } else if ((rgb_values[0] == orange[0]) && (rgb_values[1] == orange[1]) && (rgb_values[2] == orange[2])) {
    return("orange");  // orange 0, 108, 255
  } else if ((rgb_values[0] == all_off[0]) && (rgb_values[1] == all_off[1]) && (rgb_values[2] == all_off[2])) {
    return("all off");  // all off 255,255,255
  } else {
    return("unrecognized color");
  }
}

// Gives user the ability to set a known weight on the scale and calculate a calibration factor
void calibrateScale(void) {
  Serial.println();
  Serial.println();
  Serial.println(F("LC calibration"));
  Serial.print(F("Are you sure you want to calibrate? Enter y to continue, any other key to abort: "));
  readSerial();
  if ((serial_data[0] == 'y') || (serial_data[0] == 'Y')) { 
    Serial.println(F("Setup load cell with no weight on it. Press a key when ready."));
    clearSerialWait();
    load_cell.calculateZeroOffset(64); // Zero or Tare the load cell. Average over 64 readings.
    Serial.print(F("New zero offset: "));
    Serial.println(load_cell.getZeroOffset());
    // Commit zero offset to global variable
    zero_offset = load_cell.getZeroOffset();
    Serial.println(F("Place known weight on LC. Press a key."));
    clearSerialWait();
    Serial.print(F("Enter weight on the LC: "));
    clearSerialWait();
    // Read user input
    float weightOnScale = Serial.parseFloat();
    // confirm user input
    Serial.println();
    Serial.print(F("Calibration weight entered: "));
    Serial.println(weightOnScale);
    // Tell the library how much weight is currently on it
    load_cell.calculateCalibrationFactor(weightOnScale, 64); 
    Serial.println();
    Serial.print(F("New cal factor: "));
    Serial.println(load_cell.getCalibrationFactor(), 2);
    // Commit cal factor to global variable
    cal_factor = load_cell.getCalibrationFactor();
    //Serial.print(F("New Scale Reading: "));
    //Serial.println(load_cell.getWeight(), 2);
    Serial.println();
    // Commit global values to SD config.txt
    saveSystemSettings();
  } else {
    Serial.println(F("Calibration aborted"));
  }
  getCalibration();
} // End calibrateScale

// Reads the current system settings from the SD card
// If anything looks weird, reset setting to default value
// config.txt is key value declarations of variable values
void readSystemSettings(void) {
  File configFile;
  if (SD.exists("config.txt")) {
    configFile = SD.open("config.txt");
    if (configFile) {
      char buffer[40];
      byte index = 0;
      while (configFile.available()) {
       char c = configFile.read();
       if(c == '\n' || c == '\r') { // Test for <cr> and <lf>
           parseSavedVar(buffer);
           index = 0;
           buffer[index] = '\0'; // Keep buffer NULL terminated
       }
       else {
           buffer[index++] = c;
           buffer[index] = '\0'; // Keep buffer NULL terminated
       }
      }
    }
    configFile.close();
    // Set load cell to saved calibration values
    load_cell.setCalibrationFactor(cal_factor);
    load_cell.setZeroOffset(zero_offset);
  } else {
    // config.txt does not exist, create a new one with default values
    configFile = SD.open("config.txt", FILE_WRITE);
    if (configFile) {
      configFile.print("echo = "); configFile.println(DEFAULT_ECHO);
      configFile.print("log_interval = "); configFile.println(DEFAULT_LOG_INTERVAL);
      configFile.print("sync_interval = "); configFile.println(DEFAULT_SYNC_INTERVAL);
      configFile.print("cal_factor = "); configFile.println(DEFAULT_CAL_FACTOR);
      configFile.print("zero_offset = "); configFile.println(DEFAULT_ZERO_OFFSET);
      configFile.print("trip_value = "); configFile.println(DEFAULT_TRIP_VALUE);
    }
    configFile.close();
    // Re-read system settings
    readSystemSettings();
  }
  // Assume for the moment that there are good cal values
  settingsDetected = true; 
  if (cal_factor == DEFAULT_CAL_FACTOR || zero_offset == DEFAULT_ZERO_OFFSET) {
    settingsDetected = false; // Defaults detected. Prompt user to cal scale.
  }
} // End readSystemSettings

// Parse a line from config.txt to the appropriate variable, see https://forum.arduino.cc/index.php?topic=210904.0
void parseSavedVar(char *buff) {
   char *name = strtok(buff, " =");
   if (name) {
       char *junk = strtok(NULL, " ");
       if (junk) {
           char *valu = strtok(NULL, " ");
           if (valu) {
               int val = atoi(valu);
               if(strcmp(name, "echo") == 0) {
                   echo = val;
               }
               if(strcmp(name, "log_interval") == 0) {
                   log_interval = val;
               }
               if(strcmp(name, "sync_interval") == 0) {
                   sync_interval = val;
               }
               if(strcmp(name, "cal_factor") == 0) {
                   cal_factor = val;
               }
               if(strcmp(name, "trip_value") == 0) {
                   trip_value = val;
               }
               else if(strcmp(name, "zero_offset") == 0) {
                   zero_offset = val;
               }
           }
        }
     }
}

// Save the current configuration to file, when settings change
void saveSystemSettings(void) {
  File configFile;
  // Remove existing config file
  if (SD.exists("config.txt")) {
    SD.remove("config.txt");
  }
  // Recreate file and write variable values
  configFile = SD.open("config.txt", FILE_WRITE);
  if (configFile) {
      configFile.print("echo = "); configFile.println(echo);
      configFile.print("log_interval = "); configFile.println(log_interval);
      configFile.print("sync_interval = "); configFile.println(sync_interval);
      configFile.print("cal_factor = "); configFile.println(cal_factor);
      configFile.print("zero_offset = "); configFile.println(zero_offset);
      configFile.print("trip_value = "); configFile.println(trip_value);
  }
  configFile.close();
}


// Prints the current load cell calibration
void getCalibration() {
  Serial.println();
  Serial.print(F("LC 0 offset: "));
  Serial.println(load_cell.getZeroOffset());
  Serial.print(F("LC cali factor: "));
  Serial.println(load_cell.getCalibrationFactor());
  Serial.print(F("LC gain: "));
  Serial.println(gain_value_table[gain_setting]);
  Serial.print(F("LC trip value: "));
  Serial.println(DEFAULT_TRIP_VALUE);
  Serial.println();
}

// Manually calibrate load cell with known values
void manualCalibration() {
  delay(100);
  float settingCalibrationFactor; // Value used to convert the load cell reading to lbs or kg
  long settingZeroOffset; // Zero value that is found when scale is tared
  Serial.println();
  Serial.print(F("Are you sure you want to change the calibration? Enter y to continue, any other key to abort: "));
  readSerial();
  if ((serial_data[0] == 'y') || (serial_data[0] == 'Y')) { 
    Serial.println(F("Enter the 0 offset: "));
    clearSerialWait();
    zero_offset = Serial.parseFloat();
    Serial.println();
    Serial.println(F("Enter the cali factor: "));
    clearSerialWait();
    cal_factor = Serial.parseFloat();
    // Save to config.txt
    saveSystemSettings();
    // Pass these values to the library
    load_cell.setZeroOffset(zero_offset);
    load_cell.setCalibrationFactor(cal_factor);
    Serial.println(F("LC calibrated"));
    Serial.println();
  } else {
    Serial.println(F("Manual calibration update aborted"));
  }
  getCalibration();
}

// Get the current time from the real time clock as an ISO UTC char array
char * getUTC() {
  // Fetch the time
  now = rtc.now();
  // Build ISO UTC date string
  static char dtUTC[22];
  sprintf(dtUTC,"%04u-%02u-%02uT%02u:%02u:%02uZ", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return(dtUTC);
}

// Set the real time clock
void setRTC() {
  Serial.println();
  Serial.println(F("--- Set RTC ---"));
  Serial.println();
  Serial.println(F("Provide a UTC datetime."));
  delay(100); // Pause to avoid bounces
  Serial.println(F("Enter year:"));
  clearSerialWait();
  uint16_t year = Serial.parseInt();
  Serial.println(F("Enter month:"));
  clearSerialWait();
  uint8_t month = Serial.parseInt();
  Serial.println(F("Enter day:"));
  clearSerialWait();
  uint8_t day = Serial.parseInt();
  Serial.println(F("Enter hour (24 format):"));
  clearSerialWait(); 
  uint8_t hour = Serial.parseInt();
  Serial.println(F("Enter minute:"));
  clearSerialWait(); 
  uint8_t min = Serial.parseInt();
  Serial.println(F("Enter second:"));
  clearSerialWait();
  uint8_t sec = Serial.parseInt();
  Serial.println(F("Press any key when ready to set time..."));
  clearSerialWait();
  // Build datetime
  now = DateTime(year, month, day, hour, min, sec);
  // Set RTC
  rtc.adjust(now);
} // End setRTC

void setLogInterval() {
  delay(100); // Pause to avoid bounces
  Serial.println(F("Enter the LI in ms: "));
  clearSerialWait();
  // Read user input
  log_interval = Serial.parseInt();
  // Check that log_interval not greater than sync_interval
  if (log_interval > sync_interval) {
    Serial.println(F("Val is > than the sync int!"));
    setLogInterval();
    return;
  }
  // Commit values to SD config.txt
  saveSystemSettings();
  // Message
  Serial.print(F("LI set at: "));
  Serial.print(log_interval);
  Serial.println(F(" ms."));
} // End setLogInterval

void setSyncInterval() {
  delay(100); // Pause to avoid bounces
 Serial.println(F("Enter SI in ms: "));
  clearSerialWait();
  // Read user input
  sync_interval = Serial.parseInt();
  // Check that log_interval not greater than sync_interval
  if (log_interval > sync_interval) {
    Serial.println(F("Val is < than LI!"));
    setSyncInterval();
    return;
  }
  // Commit these values to SD config.txt
  saveSystemSettings();
  // Message
  Serial.print(F("SI set at: "));
  Serial.print(sync_interval);
  Serial.println(F(" ms."));
} // End setSyncInterval

// ***********************************************************************
// * SERIAL FUNCTIONS
// ***********************************************************************

// Clear the serial buffer and wait for new data
void clearSerialWait() {
  while (Serial.available()) Serial.read(); // Clear anything in RX buffer
  while (Serial.available() == 0) delay(10); // Wait for user to press key
}

// Read incoming serial data into serial_data array
void readSerial(void) {
  // Clear serial data array
  memset(serial_data, 0, sizeof(serial_data));
  int data_index = 0;
  clearSerialWait();
  // Once it is available
  while (Serial.available()) {
    char ch = Serial.read();
    //Serial.print(ch);
    if (data_index <= SERIAL_SIZE && ch != '\n' && ch != '\r' && ch != ',') {
      serial_data[data_index] = ch;
      data_index++;
    } // End if
    // Make sure serial buffer is not read too fast, can cause errors
    delay(3);
  } // End while serial available
} // End readSerial

// ***********************************************************************
// * FILE MANAGER FUNCTIONS
// ***********************************************************************

void(* resetFunc) (void) = 0; // Declare reset function at address 0

void fileManager() {
  Serial.println();
  Serial.println(F("--- FILE MANAGER ---"));
  Serial.println();
  fm = true;
  do {
    Serial.println();
    Serial.println(F("Choose: l - list files; t - transfer a file; d - delete a file; c - clear the entire SD card; x - exit file manager."));
    Serial.println(F("Enter file option:"));
    // Get incoming data
    readSerial();
    // Switch on incoming byte
    switch(serial_data[0]) {
      // List files on SD card
      case 'l': case 'L': {
        File root = SD.open("/");
        printDirectory(root, 0);
        root.close();
        break;
      }
      // Transfer file
      case 't': case 'T': {
        getFileName('t');
        break;
      }
      // Delete file
      case 'd': case 'D': {
        getFileName('d');
        break;
      }
      // Clear card
      case 'c': case 'C': {
        clearCard();
        break;
      }
      // Exit
      case 'x': case 'X': {
        fm = false;
        break;
      }
      default:
        Serial.println(F("Invalid option entered!"));
    } // End switch
  } while (fm); // End while
} // End fileManager

// Function to print the contents of a directory
// Can be called recursively to indent nested directories using numTabs arg
void printDirectory(File dir, int numTabs) {
  dir.rewindDirectory();
   while(true) {
     File entry =  dir.openNextFile();
     if (!entry) {
       // no more files
       Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

// Gets a file name from the serial terminal
void getFileName(char action) {
  Serial.println(F("Enter FN:"));
  // Get incoming data
  readSerial();
  //String readString(serial_data); 
  if (1) {
    Serial.print("FILE: ");
    Serial.println(serial_data);
    // Do something with file based on action
    switch (action) {
      // Transfer file
      case 't':
        getFile(serial_data);
        break;
      // Delete file
      case 'd':
        delFile(serial_data);
        break;
    } // End switch
  } // End If
} // End getFileName


// Transfers a CSV file over serial
void getFile(char* fn) {
  Serial.println();
  // Check that file exists
  if (!SD.exists(fn)) {
    Serial.println(F("File does not exist."));
    return;
  }
  // Open file
  File dumpFile = SD.open(fn);
  if (dumpFile) {
    Serial.print(F("File dump from "));
    Serial.println(fn);
    Serial.println();
    Serial.println(F("--------------------------"));
    Serial.println();
    // Dump file to serial
    while (dumpFile.available()) {
      Serial.write(dumpFile.read());
    }
    // Close file
    dumpFile.close();
    Serial.println();
    Serial.println(F("--------------------------"));
    Serial.println();
    Serial.println(F("Done!")); 
    
  } else {
    Serial.println(F("Error opening file."));
  }
} // End getFile

// Delete file
void delFile(char* fn) {
  // Check that file exists
  if (!SD.exists(fn)) {
    Serial.println(F("File entered does not exist."));
    return;
  }
  // Delete file
  if (SD.remove(fn)) {
    Serial.println(F("File removed."));
  } else {
    Serial.println(F("File could not be removed."));
  }
  Serial.println();
} // End delFile

// Clear all files on the SD card
void clearCard() {
  Serial.println();
  Serial.println(F("WARNING: All data on card will be cleared - type Y to continue, or any other key to abort."));
  // Get incoming data
  readSerial();
  // If anything but Y sent, abort
  if(!(serial_data[0] == 'Y' || serial_data[0] == 'y')) {return;}
  // Proceed with card clear
  File root = SD.open("/");
  root.rewindDirectory();
  while (true) {
    File entry =  root.openNextFile();
    if (!entry) {
      // No more files
      break;
    }
    // Skip current logfile TODO not working as expected
    if (strcmp(entry.name(), filename) == 0) {entry.close(); continue;}
    // Skip config file
    if (strcmp(entry.name(), "CONFIG.TXT") == 0) {entry.close(); continue;}
    Serial.print(entry.name());
    // Delete file
    if (SD.remove(entry.name())) {
      Serial.println(F(" removed."));
    } else {
      Serial.println(F(" could not be removed."));
    }
    entry.close();
  } // End while
  // Restart logger
  //resetFunc();
} // End clearCard

// Error handler, called anytime an error/panic is hit. Stops everything. 
void error(const __FlashStringHelper*err) {
  Serial.print(err);
  Serial.println(" error");
  // Write RX led low to turn on
  digitalWrite(ERROR_LED, LOW);
  // Set RGB to magenta
  // setRGB(255, 0, 255);  
  // setRGB(0, 255, 0);  // common anode LED magenta
  setRGB(magenta, 3);
  // Halts program execution
  Serial.println(F("Program suspended"));
  while(1);
} // End error