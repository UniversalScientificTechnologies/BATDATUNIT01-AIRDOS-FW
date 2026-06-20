#define TYPE "AIRDOS04C"
#define DIGTYPE "BATDATUNIT01B"
#define ADCTYPE "USTSIPIN03A"
// Compiled with: Arduino 1.8.13
// MightyCore 2.2.2

#define MAJOR 2   // Data format
#define MINOR 0   // Features

#define DEBUG

#define XSTR(s) STR(s)
#define STR(s) #s

#define CHANNELS 4 // number of channels in the buffer for histogram
#define MAX_EVENTS 300 // number of events per integration time



#define MAXFILESIZE MAX_MEASUREMENTS * BYTES_MEASUREMENT // in bytes, 4 MB per day, 28 MB per week, 122 MB per month
#define MAX_MEASUREMENTS 11000ul // in measurement cycles, 5 500 per day
#define BYTES_MEASUREMENT 400ul // number of bytes per one measurement
#define MAXFILES 200 // maximal number of files on the SD card
#define _5S 39063  // ticks of timer during 5 s

/*
ISP
---
PD0     RX
PD1     TX
RESET#  through 50M capacitor to RST#


                     Mighty 1284p
                      +---\/---+
           (D 0) PB0 1|        |40 PA0 (AI 0 / D24)
           (D 1) PB1 2|        |39 PA1 (AI 1 / D25)
      INT2 (D 2) PB2 3|        |38 PA2 (AI 2 / D26)
       PWM (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
    PWM/SS (D 4) PB4 5|        |36 PA4 (AI 4 / D28)
      MOSI (D 5) PB5 6|        |35 PA5 (AI 5 / D29)
  PWM/MISO (D 6) PB6 7|        |34 PA6 (AI 6 / D30)
   PWM/SCK (D 7) PB7 8|        |33 PA7 (AI 7 / D31)
                 RST 9|        |32 AREF
                VCC 10|        |31 GND
                GND 11|        |30 AVCC
              XTAL2 12|        |29 PC7 (D 23)
              XTAL1 13|        |28 PC6 (D 22)
      RX0 (D 8) PD0 14|        |27 PC5 (D 21) TDI
      TX0 (D 9) PD1 15|        |26 PC4 (D 20) TDO
RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS
TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK
     PWM (D 12) PD4 18|        |23 PC1 (D 17) SDA
     PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL
     PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM
                      +--------+
*/

/*
Using library Wire at version 1.1 in folder: /home/kacer/.arduino15/packages/MightyCore/hardware/avr/2.2.2/libraries/Wire
Using library SD at version 1.2.4 in folder: /home/kacer/Arduino/libraries/SD
Using library SPI at version 1.0 in folder: /home/kacer/.arduino15/packages/MightyCore/hardware/avr/2.2.2/libraries/SPI

Using library SHT31 at version 0.5.0 in folder: /home/kacer/Arduino/libraries/SHT31
https://github.com/RobTillaart/SHT31

Using library MS5611 at version 0.4.0 in folder: /home/kacer/Arduino/libraries/MS5611
https://github.com/RobTillaart/MS5611

 */

#include "wiring_private.h"
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SHT31.h>
#include <MS5611.h>
#include <avr/wdt.h>
#include "../../xDOS-versions/generated/eeprom_layout.h"
#include "githash.h"

String FWversion = XSTR(MAJOR)"."XSTR(MINOR)"."XSTR(GHRELEASE)"-"XSTR(GHBUILD)"-"XSTR(GHBUILDTYPE);

#define CONV        0    // PB0, MOLEX B0, D Q, ADC CONV signal
#define DRESET      22   // PC6, MOLEX C0, D #Reset
#define DSET        23   // PC7, MOLEX C1, D #Set
#define SDpower     19   // PC3
#define SDmode      3    // PB3
#define SS          4    // PB4
#define MOSI        5    // PB5
#define MISO        6    // PB6
#define SCK         7    // PB7
#define LED1        12   // PD4
#define LED2        13   // PD5
#define LED3        14   // PD6
#define BUZZER      15   // PD7
#define POWER5V     26   // PA2
#define POWER3V3    2    // PB2
#define SPI_MUX_SEL 18   // PC2
#define EXT_I2C_EN  20   // PC4
#define ACONNECT    27   // PA3 = LOW = analogue frontend connected
#define CTS         28   // PA4
#define RTS         29   // PA5
#define BTN_USER_A  30   // PA6
//#define BTN_USER_B  31   // PA7
#define ENUM_FTDI_USB 21 // PC5 = LOW = USB connected

#define BQ34Z100 0x55
#define SD_READER_ADDR 0x71
#define EEPROM_ANALOG_SN_ADDR 0x5B
#define EEPROM_DIGITAL_SN_ADDR 0x58
#define EEPROM_DIGITAL_CFG_ADDR 0x50
#define EEPROM_ANALOG_CFG_ADDR 0x53
#define SHT31_ADDR_PRIMARY 0x44
#define SHT31_ADDR_SECONDARY 0x45
#define MS5611_ADDR 0x77
#define VBAT_ADC_LSB_0P01_MV     199u      // 1.99 mV = 199 × 0.01 mV
#define VBAT_PRESENT_THRESHOLD_MV 1500u    // 1.5 V jako hranice „baterie je přítomna“ - pod hranicí 2.2V což je limit vypnutí napájení chargeru

#include "rtc_impl.h"
#include "charger_impl.h"
#include "eeprom_impl.h"
#include "tone_led_impl.h"
#include "i2c_switch_impl.h"

String filename = "";
uint16_t fn;
uint16_t count = 0;
boolean SDinserted = true;
String logHeader = "";              // Device identification header ($DOS, $DIG, $ADC, $BATP); $TIME is appended fresh per file
uint32_t measurement_start_unix_time = 0; // Unix timestamp of measurement start (for $FSEQ)
uint16_t file_seq = 0;              // File sequence number within one measurement (0 = first file)
uint16_t histogram[CHANNELS];
uint16_t event_time[MAX_EVENTS];
uint8_t event_time2[MAX_EVENTS];
uint16_t event_channel[MAX_EVENTS];
uint16_t events_counter;
boolean clicks = false;
bool batteryPresent = true;
uint16_t detectedBatteryMv = 0;

void(* resetFunc) (void) = 0; //declare reset function at address 0

void enableSD()
{
  digitalWrite(SDpower, HIGH);   // SD card power on
  digitalWrite(SPI_MUX_SEL, LOW); // SDcard
}

void disableSD()
{
  digitalWrite(SPI_MUX_SEL, HIGH); // ADC
  digitalWrite(SDpower, LOW);   // SD card power off
  delay(1);
}

void writeToSD(const String &dataString)
{
  if (SDinserted)
  {
    digitalWrite(LED3, HIGH);
    // make sure that the default chip select pin is set to output
    // see if the card is present and can be initialized:
    if (!SD.begin(SS))//, SPI_HALF_SPEED))
    {
      Serial1.println("#SD init false");
      SDinserted = false;
      // don't do anything more:
    }
    else
    {
      // open the file. note that only one file can be open at a time,
      // so you have to close this one before opening another.
      File dataFile = SD.open(filename, FILE_WRITE);

      // if the file is available, write to it:
      if (dataFile)
      {
        dataFile.println(dataString);  // write to SDcard (800 ms)
        dataFile.close();
      }
      // if the file isn't open, pop up an error:
      else
      {
        Serial1.println("#SD false");
        SDinserted = false;
      }
    }

    digitalWrite(SS, HIGH);    // Disable SD card     
    digitalWrite(LED3, LOW);
  }
}


// Verify RTC registers: reg 0x07 bit7 == 0 and reg 0x28 bit4 (RTCM) == 1.
// If not in required mode, initialize to stopwatch and log the change.
void CheckRTCconfig()
{
  uint8_t r07 = readRTCreg(0x07);
  uint8_t r28 = readRTCreg(0x28);

  bool ok07 = ((r07 & 0x80) == 0); // bit7 must be zero
  bool okRTCM = ((r28 & (1<<4)) != 0); // RTCM (bit4) must be 1

  if (!ok07 || !okRTCM)
  {
    ResetRTC();
    // Log action to SD card and UART1
    readRTC();
    String s = "$RTCCHK,";
    s += String(rtc_current_time_s);
    s += ".";
    s += String(rtc_current_time_s100);
    s += ",INIT,reg07=0x";
    if (r07 < 16) s += "0";
    s += String(r07, HEX);
    s += ",reg28=0x";
    if (r28 < 16) s += "0";
    s += String(r28, HEX);

    Serial1.println(s);  // Output to debug UART
    writeToSD(s);
  } else {
    // RTC config is OK - log to SD and UART1
    readRTC();
    String s = "$RTCCHK,";
    s += String(rtc_current_time_s);
    s += ".";
    s += String(rtc_current_time_s100);
    s += ",OK,reg07=0x";
    if (r07 < 16) s += "0";
    s += String(r07, HEX);
    s += ",reg28=0x";
    if (r28 < 16) s += "0";
    s += String(r28, HEX);

    Serial1.println(s);  // Output to debug UART
    writeToSD(s);
  }
}

// Build a fresh $TIME line from the current RTC reading. Called both for the
// initial log header and on each file rotation so the timestamp reflects when
// the file was actually created, not when the measurement started.
void appendTimeLine(String &s)
{
  readRTC();
  uint32_t current_unix_time = 0;
  uint32_t sync_age = 0;
  if (eeprom_sync_time > 0)
  {
    current_unix_time = getCurrentUnixTime();
    sync_age = rtc_current_time_s - eeprom_sync_rtc_seconds;
  }

  s += "\r\n$TIME,";
  s += String(rtc_current_time_s);
  s += ",";
  s += String(eeprom_sync_time);
  s += ",";
  s += String(current_unix_time);
  s += ",";
  s += String(sync_age);
  s += ",";
  addHumanReadebleFromUnixTime(current_unix_time,s);
}

uint8_t store = 0;
uint8_t batt = 0;
uint8_t env = 0;



// Timer 1 interrupt service routine (ISR)
ISR(TIMER1_COMPA_vect)
{
  store++;
  TCNT1 = 0; // Reset Counter
}

// Enviromental sensors out
void EnvOut()
{
  wdt_enable(WDTO_2S);  // watchdog for preventing I2C hanging

  enableSD();

  digitalWrite(POWER3V3, HIGH); // Analog power 3.3 V on 
  digitalWrite(EXT_I2C_EN, HIGH);// Enable external I2C

  // make a string for assembling the data to log:
  String dataString = "";

  readRTC();

  // make a string for assembling the data to log:
  dataString += "$ENV,";
  dataString += String(count);
  dataString += ",";
  dataString += String(rtc_current_time_s);
  dataString += ".";
  dataString += String(rtc_current_time_s100);
  dataString += ",";

  SHT31 sht(SHT31_ADDR_PRIMARY);
  sht.begin();
  sht.read();         //  default = true/fast       slow = false

  dataString += String(sht.getTemperature(), 1);
  dataString += String(",");
  dataString += String(sht.getHumidity(), 1);
  dataString += String(",");

  SHT31 sht2(SHT31_ADDR_SECONDARY);
  sht2.begin();
  sht2.read();         //  default = true/fast       slow = false

  dataString += String(sht2.getTemperature(), 1);
  dataString += String(",");
  dataString += String(sht2.getHumidity(), 1);
  dataString += String(",");

  MS5611 MS5611(MS5611_ADDR);
  MS5611.begin();
  MS5611.read();           //  note no error checking => "optimistic".

  dataString += String(MS5611.getTemperature(), 2);
  dataString += String(",");
  dataString += String(MS5611.getPressure(), 2);
  
  digitalWrite(EXT_I2C_EN, LOW);  // Disable external I2C
  digitalWrite(POWER3V3, LOW);  // Analog power 3.3 V off

  wdt_disable();

  #ifdef DEBUG
    Serial1.println(dataString);  // Debug output to debug terminal        
  #endif

  writeToSD(dataString);
  disableSD();

}

// Battery status out
void BattOut()
{
  enableSD();

  // make a string for assembling the data to log:
  String dataString = "";

  readRTC();

  // make a string for assembling the data to log:
  dataString += "$BATT,";
  dataString += String(count);
  dataString += ",";
  dataString += String(rtc_current_time_s);
  dataString += ".";
  dataString += String(rtc_current_time_s100);
  dataString += ",";
  dataString += String(readBat(0x8));   // mV - U
  dataString += ",";
  dataString += String(readBat(0xa));  // mA - I
  dataString += ",";
  dataString += String(readBat(0x4));   // mAh - remaining capacity
  dataString += ",";
  dataString += String(readBat(0x6));   // mAh - full charge
  dataString += ",";
  dataString += String(readBat(0xc) * 0.1 - 273.15);   // temperature

  #ifdef DEBUG
    Serial1.println(dataString);  // Debug output to debug terminal        
  #endif

  writeToSD(dataString);
  disableSD();
}

// Data out
void DataOut()
{
  enableSD();

  readRTC();

  // make a string for assembling the data to log:
  String dataString = "";
  
  // Output of system time of stat the integration
  dataString += "$START,";
  dataString += String(count);
  dataString += ",";
  dataString += String(event_time[0]);

  // Evens out
  for(uint16_t n=1; n<events_counter; n++)
  {
     if (n>=MAX_EVENTS) break;
     uint32_t long_event_time;
     if (0 == event_time2[n])  
     {
       //long_event_time =  2 * _5S - event_time[n];
       long_event_time =  event_time[n];
     }
     else
     {
       //long_event_time =  _5S - event_time[n];
       long_event_time =  _5S + event_time[n];
     };
     
     dataString += "\r\n";
     dataString += "$E,";
     dataString += String(long_event_time);
     dataString += ",";
     dataString += String(event_channel[n]);
  }

  
  // End of integration
  dataString += "\r\n$STOP,";
  dataString += String(count);
  dataString += ",";
  dataString += String(rtc_current_time_s);
  dataString += ".";
  dataString += String(rtc_current_time_s100);
  dataString += ",";
  {
    uint16_t systime = TCNT3L; // read system time
    systime |= TCNT3H<<8;
    dataString += String(systime);
  };
  dataString += ",";
  dataString += String(events_counter-1);

  for(uint16_t n=0; n<CHANNELS; n++)
  {
    dataString += ",";
    dataString += String(histogram[n]);
  }


  #ifdef DEBUG
    Serial1.println(dataString);  // Debug output to debug terminal        
  #endif

  writeToSD(dataString);

  if(!SDinserted)
  {
    digitalWrite(LED2, HIGH);
    // Debug output if SD card is not inserted
    uint16_t i=0;
    uint16_t len = dataString.length();
    while(true)
    {
      for(uint8_t n=0; n<255; n++) if (!digitalRead(RTS)) break;
      {delayMicroseconds(50);Serial.print(dataString[i++]);}
      if (i>len) break;
      //delay(2);
    }
    Serial.println();             // print to HID
    Serial1.println(dataString);  // print to debug terminal
  }
  digitalWrite(LED3, LOW);
  digitalWrite(LED2, LOW);

  if (count > MAX_MEASUREMENTS)
  {
    count = 0;
    fn++;
    filename = String(fn) + ".TXT";
    file_seq++;
    Serial1.print("#Filename,");
    Serial1.println(filename);

    // Write detector identification header + $FSEQ line to the new file,
    // so each rotated file can be identified on its own.
    String hdr = logHeader;
    appendTimeLine(hdr);
    hdr += "\r\n$FSEQ,";
    hdr += String(file_seq);
    hdr += ",";
    hdr += String(measurement_start_unix_time);

    writeToSD(hdr);    
  }

  disableSD();

}

// Power off without discharging C21: power-off tone, then disable charger BATFET
// (ship mode). Used for intentional power-off by button - skipping the C21 discharge
// keeps the QON node high so the device stays off (see powerOffWithDischarge).
//
// On battery power the MCU loses power inside this function and never returns. If external
// (USB) power is present the charger ignores the shutdown and the MCU keeps running; in that
// case this function waits a few seconds and then returns, so the caller can resume normal
// operation instead of being left in an undefined state where it might corrupt data.
void powerOff()
{
    i2c_switch_forceTakeBus();

    wdt_reset();
    playPowerOffTone();

    // Disable charger BATFET (ship mode) - removes power when running from battery.
    Wire.beginTransmission(CHARGER_ADDR); // I2C address
    Wire.write((uint8_t)0x18); // Start register
    Wire.write((uint8_t)0x0A); //
    Wire.endTransmission();

    // Give the power path time to actually cut power. Keep the watchdog fed so we do not
    // reset mid-shutdown. If we are still running after this, the shutdown was ignored
    // (external power present) - return so the caller can resume normal operation.
    for(uint8_t i=0; i<5; i++)
    {
      wdt_reset();
      delay(1000);
    }

    Serial1.println("#PowerOff ignored (external power present), resuming");
    wdt_reset();
}

// Discharge C21 via 1024 Hz on the RTC INTA pin, then power off.
// Needed only when the connector is disconnected (EXT_DETECTION floating).
void powerOffWithDischarge()
{
    i2c_switch_forceTakeBus();


    Wire.beginTransmission(RTC_ADDR); // 1024 Hz to #INTA
    Wire.write((uint8_t)0x27); // Start register
    Wire.write((uint8_t)0x00); // 0x27 Enable CLX output on INTA pin, using bits set in reg 0x28
    Wire.write(0x95);             // COF
    Wire.endTransmission();

    wdt_reset();
    delay(1000); // Vaiting for capacitor discharge
    wdt_reset();
    delay(1000);
    wdt_reset();
    delay(1000);
    wdt_reset();

    Wire.beginTransmission(RTC_ADDR); // High-Z on #INTA
    Wire.write((uint8_t)0x27); // Start register
    Wire.write((uint8_t)0x03); // 0x27 High-Z on INTA pin.
    Wire.write(0x95);             // COF
    Wire.endTransmission();

    powerOff();
}

void powerOffByButton()
{
  uint8_t led1=digitalRead(LED1);
  uint8_t led2=digitalRead(LED2);
  uint8_t led3=digitalRead(LED3); 

  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(LED3, HIGH);

  uint32_t startMs = millis();
  bool armed = false;       // set once the 3 s countdown completes
  while(!digitalRead(BTN_USER_A))
  {
      uint32_t nowMs = millis();
      if(nowMs%500==0)
        wdt_reset();
      if(nowMs-startMs>1000)
        digitalWrite(LED1, LOW);

      if(nowMs-startMs>2000)
        digitalWrite(LED2, LOW);
      if(nowMs-startMs>3000)
      {
        digitalWrite(LED3, LOW);
        armed = true;
      }
      if(nowMs-startMs>15000) // button stuck/blocked: abort, do not power off
      {
        Serial1.println("#PowerOff aborted: button held too long");
        digitalWrite(LED1, led1);
        digitalWrite(LED2, led2);
        digitalWrite(LED3, led3);
        return;
      }
  }

  // Button released. Power off only if the countdown completed - this requires a
  // deliberate press-and-release, so a stuck/blocked button never powers the device off.
  if(armed)
  {
    Serial1.println("#PowerOff by button");
    powerOff(); // returns only if shutdown was ignored (external power present)
  }

  // Reached if released before the countdown finished, or if powerOff() returned because
  // external power kept the device on. Restore LED state and resume normal operation.
  digitalWrite(LED1, led1);
  digitalWrite(LED2, led2);
  digitalWrite(LED3, led3);
}

//file log rotation logic.
//filename,fn <>
//logHeader >
void updateUsedFile()
{

  // make sure that the default chip select pin is set to output
  // see if the card is present and can be initialized:
  if (!SD.begin(SS))//, SPI_HALF_SPEED))
  {
    Serial1.println("#SD init false");
    SDinserted = false;
  }
  for (fn = 1; fn<MAXFILES; fn++) // find last file -- tohle najde nenižší neexistující, trochu riskantní
  {
     filename = String(fn) + ".TXT";
     if (SD.exists(filename) == 0) break;
  }
  
  if(fn==MAXFILES)
  {
    Serial1.println("#No available File, MAXFILES reached");
    //fatal error...
  }

  blinkLeds(false,true,false,80,5);
  digitalWrite(LED2, HIGH);

  Serial1.print("#Filename,");
  Serial1.println(filename);
  //SD.end()


  String dataString = logHeader;
  appendTimeLine(dataString);

  // Append $FSEQ line to the first file header (file sequence 0, measurement start unix time)
  dataString += "\r\n$FSEQ,";
  dataString += String(file_seq);
  dataString += ",";
  dataString += String(measurement_start_unix_time);

  writeToSD(dataString);

}

//když je odpojena analogová část:
// - bud je připojené USB 
//      - tlačítem se mění se mody:
//           - card reader
//           - usb-hid
// - nebo usb připojené není a pak se zařízení za chvilku vypne.
// když jsme v modu usb-hid a obslužný program přebral správu I2C, tak nedovedeme číst napětí z CHARGERU
// pokud obslužný sw nepřepne,lze vypnout jen restartem
void usbServiceMode()
{
    blinkLeds(true,false,false,80,5);
    digitalWrite(LED1, HIGH);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);

    bool UsbModeSDreader = true;    // wanted SD reader mode
    bool UsbModeChanged = true;  // USB device need to be changed
    uint32_t usbLedTickMs = 0;
    bool i2c_switch_to_usb = !i2c_switch_isBusMine();

    wdt_enable(WDTO_2S);  // watchdog for preventing I2C hanging
    
    if(!i2c_switch_to_usb)
      getChargerVBUSVoltage();

    wdt_reset();
    delay(1000); // Vaiting for stable voltage
    wdt_reset();
    delay(1000); // Vaiting for stable voltage
    wdt_reset();
    delay(1000); // Vaiting for stable voltage
    wdt_reset();

    uint8_t cable_disconected=0;
    while(true)
    {      
      uint8_t switch_status;
      bool usb_phy_connected = (digitalRead(ENUM_FTDI_USB) == LOW);
      i2c_switch_to_usb = !i2c_switch_isBusMine();
      digitalWrite(LED2, i2c_switch_to_usb); //red led when I2C is not mmine
      

      // USB mode LED indication:
      // USB-SD  -> slow blink on LED1
      // USB-I2C -> double blink on LED2+LED3
     /*uint32_t nowMs = millis();
      if (UsbModeSDreader)
      {
        bool ledOn = (nowMs%800ul) >= 400ul;        
        digitalWrite(LED1, ledOn ? HIGH : LOW);
        digitalWrite(LED2, LOW);
        digitalWrite(LED3, LOW);
      }
      else
      {
        uint16_t phase = (uint16_t)(nowMs % 800ul);
        bool ledOn = (phase < 100) || ((phase >= 200) && (phase < 300));
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, ledOn ? HIGH : LOW);
        digitalWrite(LED3, ledOn ? HIGH : LOW);
      }*/

      wdt_reset();
      
      // Never evaluate local power-off while USB cable is physically connected
      // or while USB master owns I2C switch (USB-I2C mode).
      if (!usb_phy_connected && !i2c_switch_to_usb)
      {
        // Is VBUS (USB) present?
        uint8_t vbus = getChargerVBUSVoltage();
        Serial1.println(vbus);
        if (vbus < 17) // < 4.5 V
        {
          cable_disconected++;
          if(cable_disconected >10)
          {
            Serial1.println("#PowerOff by usb disconected");
            return; //No power on vbus - end.
          }
        }
        else
        {
          cable_disconected=0;
        }
      }
      
      wdt_reset();

      if (UsbModeChanged)
      {
        UsbModeChanged = false;
        playUSBModeChangeTone();          // Signal mode change to user
        
        if (UsbModeSDreader)
        {
          digitalWrite(LED3, HIGH);

          // USB-SD mode: keep external 3V3 I2C supply off.
        
          digitalWrite(EXT_I2C_EN, LOW);

          // SD card reader ON
          digitalWrite(SDmode, HIGH);   // SD card reader oscilator on
          
          // SD card reader on (charger stays as default enabled)
          Wire.beginTransmission(SD_READER_ADDR); // card reader address
          Wire.write((uint8_t)0x00); // Start register
          Wire.write((uint8_t)0b00010011); // 0b0001 0 01 1
          Wire.endTransmission();
        }
        else
        {
          digitalWrite(LED3, LOW);

          // USB-I2C mode: enable external 3V3 I2C supply.
          //digitalWrite(POWER3V3, HIGH);    // Analog power 3.3 V 
          //digitalWrite(POWER5V, HIGH);     // Analog power 5 V
          //on on boot

          digitalWrite(EXT_I2C_EN, HIGH);

          // SD card reader off
          Wire.beginTransmission(SD_READER_ADDR); // card reader address
          Wire.write((uint8_t)0x00); // Start register
          Wire.write((uint8_t)0b00010000); // 0b0001 0 00 0
          Wire.endTransmission();
          // SD card reader OFF
          digitalWrite(SDmode, LOW);   // SD card reader oscilator off
        }
        delay(1000);
      };

      if (!digitalRead(BTN_USER_A))
      {
        UsbModeSDreader = !UsbModeSDreader;
        UsbModeChanged = true;
        powerOffByButton();
      }
    }

}

void setup()
{
  //disable watchdog from previous run:
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  Serial1.begin(115200);
  Wire.setClock(100000);

  Serial1.println("#Cvak...");
  Serial1.print("#Reset bits: (JTAG,Watchdog,Low Voltage, Reset Pin, Power On): ");
  Serial1.println(mcusr,BIN);

  Serial1.print("#I2C: Own Bus:");
  Serial1.println(i2c_switch_isBusMine(),BIN);

  Serial1.print("#I2C Force Bus Owner:");
  Serial1.println(i2c_switch_forceTakeBus(),BIN);

  pinMode(ACONNECT, INPUT);      // detection of analog frontend
  pinMode(ENUM_FTDI_USB, INPUT); // detection of USB
  pinMode(RTS, INPUT);           // UART handshake

  pinMode(BTN_USER_A, INPUT);   // Button st the front panel

  pinMode(DRESET, OUTPUT);   // peak detetor
  pinMode(DSET, OUTPUT);
  pinMode(CONV, INPUT);

  pinMode(BUZZER, OUTPUT); // Set the buzzer pin as an output
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SPI_MUX_SEL, OUTPUT);   // SDcard/ADC
  digitalWrite(SPI_MUX_SEL, HIGH); // ADC

  pinMode(EXT_I2C_EN, OUTPUT); 

  pinMode(POWER3V3, OUTPUT);    // Analog power 3.3 V
  digitalWrite(POWER3V3, HIGH); // on
  pinMode(POWER5V, OUTPUT);     // Analog power 5 V
  digitalWrite(POWER5V, HIGH);  // on

  pinMode(SDpower, OUTPUT);  // SDcard interface
  pinMode(SDmode, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(MOSI, INPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);

  digitalWrite(SDpower, HIGH);   // SD card power on
  digitalWrite(SS, HIGH);        // Disable SD card
  digitalWrite(SCK, LOW);        
  delay(100); 

  digitalWrite(SDmode, LOW);     // SD card reader oscilator off

  pinMode(EXT_I2C_EN, OUTPUT);    // Disable external I2C
  digitalWrite(EXT_I2C_EN, LOW);

  // Detect battery presence and configure charger accordingly
  configCharger(false);
  delay(100);
  batteryPresent = detectBatteryPresence(detectedBatteryMv);
  configCharger(batteryPresent);
  
  if (digitalRead(ACONNECT) || !digitalRead(BTN_USER_A) )  // Analog board disconnected, or button for usbServiceMode
  {
    usbServiceMode();    
    powerOffWithDischarge();        
    while(true); // Waiting for reset

  }

  digitalWrite(EXT_I2C_EN, HIGH); // Enable external I2C

  blinkLeds(true,false,false,80,5);
  digitalWrite(LED1, HIGH);
  playPowerOnTone();

  wdt_reset();

  Serial1.println("#Hmmm...");

  digitalWrite(DSET, LOW);       // Disable ADC
  digitalWrite(DRESET, HIGH);


  Wire.beginTransmission(RTC_ADDR); // disable output n INTA
  Wire.write((uint8_t)0x27); // Start register
  Wire.write((uint8_t)0x03); // 0x27 High-Z on INTA pin
  Wire.write((uint8_t)0x97); // 0x28 stop-watch mode, no periodic interrupts, INTA in high-Z
  Wire.endTransmission();

  wdt_reset();

  // Initiation of RTC
  Wire.beginTransmission(RTC_ADDR); // init clock
  Wire.write((uint8_t)0x23); // Start register
  Wire.write((uint8_t)0x00); // 0x23
  Wire.write((uint8_t)0x00); // 0x24 Two's complement offset value
  Wire.write((uint8_t)0b00000101); // 0x25 Normal offset correction, disable low-jitter mode, set load caps to 6 pF
  Wire.write((uint8_t)0x00); // 0x26 Battery switch reg, same as after a reset
  Wire.write((uint8_t)0x03); // 0x27 Enable CLK pin, using bits set in reg 0x28
  Wire.write((uint8_t)0x97); // 0x28 stop watch mode, no periodic interrupts, CLK pin off
  Wire.write((uint8_t)0x00); // 0x29
  Wire.write((uint8_t)0x00); // 0x2a
  Wire.endTransmission();
  
  wdt_disable();

  wdt_enable(WDTO_8S);  // watchdog for preventing I2C hanging

  // Read current RTC time
  readRTC();
  Serial1.print("#RTC_TIME,");
  Serial1.println(rtc_current_time_s);

  // Read sync_time and device name from internal EEPROM (digital cfg)
  DosimeterEeprom eeprom_record = {};
  if (readEEPROMRecord(EEPROM_DIGITAL_CFG_ADDR, eeprom_record))
  {
    eeprom_sync_time = eeprom_record.rtc_history[0].reference_timestamp;
    eeprom_sync_rtc_seconds = eeprom_record.rtc_history[0].rtc_value_at_reference_timestamp;
  }
  else
  {
    Serial1.println("#EEPROM read failed");
    eeprom_sync_time = 0;
    eeprom_sync_rtc_seconds = 0;
  }

  // Read device name from analog cfg EEPROM
  DosimeterEeprom eeprom_adc_record = {};
  if (readEEPROMRecord(EEPROM_ANALOG_CFG_ADDR, eeprom_adc_record))
  {

  }
  else
  {
    Serial1.println("#EEPROM analog read failed");
  }


  enableSD();

  // make logfile header
  logHeader = "$DOS,"TYPE"," + FWversion + ",0," + githash + ","; // FW version and Git hash
  logHeader += "\r\n$DIG,"DIGTYPE",";
  readEepromSNToString(EEPROM_DIGITAL_SN_ADDR,logHeader); //DIGITAL SN to Output
  logHeader += ",";

  logHeader += String(((uint8_t*)&eeprom_record)[0],HEX); //first byte of eeprom - //DIGconf1 
  logHeader += String(((uint8_t*)&eeprom_record)[1],HEX); //second byte of eeprom - //DIGconf2 
  

  // Digital module name from EEPROM digital cfg (up to 10 chars, may not be null-terminated).
  // Empty if EEPROM read failed (eeprom_record is zero-initialized).
  logHeader += "\r\n$DIG_NAME,";
  for (uint8_t i = 0; i < sizeof(eeprom_record.device_identifier); i++)
  {
    char c = eeprom_record.device_identifier[i];
    if (c == '\0') break;
    logHeader += c;
  }

  logHeader += "\r\n$ADC,"ADCTYPE",";
  readEepromSNToString(EEPROM_ANALOG_SN_ADDR,logHeader); //ANALOG SN to Output
  logHeader += ",";

  
  logHeader += String(((uint8_t*)&eeprom_adc_record)[0],HEX); //first byte of eeprom - //ADCconf1 
  logHeader += String(((uint8_t*)&eeprom_adc_record)[1],HEX); //second byte of eeprom - //ADCconf2 

  // Analog module name from EEPROM analog cfg (up to 10 chars, may not be null-terminated).
  // Empty if EEPROM read failed (eeprom_adc_record is zero-initialized).
  logHeader += "\r\n$ADC_NAME,";
  for (uint8_t i = 0; i < sizeof(eeprom_adc_record.device_identifier); i++)
  {
    char c = eeprom_adc_record.device_identifier[i];
    if (c == '\0') break;
    logHeader += c;
  }

  // Calibration coefficients from analog board EEPROM
  logHeader += "\r\n$CALIB,";
  logHeader += String(eeprom_adc_record.calibration_constants[0], 6);
  logHeader += ",";
  logHeader += String(eeprom_adc_record.calibration_constants[1], 6);
  logHeader += ",";
  logHeader += String(eeprom_adc_record.calibration_constants[2], 6);
  logHeader += ",";
  logHeader += String(eeprom_adc_record.calibration_version);

  logHeader += "\r\n$BATP,";
  logHeader += batteryPresent ? "1" : "0";
  logHeader += ",";
  logHeader += String(detectedBatteryMv); //tohle se neaktualizuje a je to v halvičce..

  Serial1.println(logHeader);  // print header

  measurement_start_unix_time = getCurrentUnixTime();
  file_seq = 0;
  updateUsedFile();

  // Check RTC registers and initialize to stopwatch mode if necessary.
  // SD is already initialized here and `filename` set, so logging is possible.
  CheckRTCconfig();

  disableSD();
  
   
  digitalWrite(EXT_I2C_EN, LOW); // Disable external I2C
  blinkLeds(false,false,true,80,5);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);

  digitalWrite(POWER3V3, LOW); // Analog power 3.3 V off

  wdt_disable();

  cli(); // disable interrupts during setup
  // Configure Timer 1 interrupt
  // F_clock = 8 MHz, prescaler = 1024, Fs = 7.8125 kHz (128 us / tick)
  TCCR1A = 0;
  //TCCR1B = 1<<WGM12 | 0<<CS12 | 1<<CS11 | 1<<CS10;
  TCCR1B = 1<<WGM12 | 1<<CS12 | 0<<CS11 | 1<<CS10;
  // OCR1A = ((F_clock / prescaler) / Fs) - 1
  OCR1A = _5S;      // Set sampling frequency Fs, period 5 s
  TCNT1 = 0;          // reset Timer 1 counter
  TIMSK1 = 1<<OCIE1A; // Enable Timer 1 interrupt

  // Configure Timer 3 interrupt
  // F_clock = 8 MHz, prescaler = 1024, Fs = 7.8125 kHz (128 us / tick)
  TCCR3A = 0;
  TCCR3B = 1<<CS32 | 0<<CS31 | 1<<CS30;
  TCNT3H = 0;          // reset Timer 1 counter
  TCNT3L = 0;          // reset Timer 1 counter
  
  sei(); // re-enable interrupts
}

inline void PostIntegration() __attribute__((always_inline));

inline void PostIntegration()
{
  store = 0;
  batt++;
  env++;

  digitalWrite(LED2, digitalRead(ACONNECT));
  if (digitalRead(ACONNECT))  // Analog part is disconnected?
  {
    powerOffWithDischarge();
  }

  digitalWrite(DRESET, HIGH);
  digitalWrite(DSET, LOW);

  wdt_enable(WDTO_8S);  // watchdog for preventing I2C hanging

  DataOut();  // Save data from integration time
  for(int n=0; n<CHANNELS; n++) // reset histogram
  {
    histogram[n]=0;
  };
  events_counter = 1; // Start events from 1 (because 0 is record of death time)

  if (env >= 5*6) // Environment out every 5 minutes
  {
    env = 0;
    EnvOut();
  };

  if (batt >= 30*6) // Battery status every 30 minutes
  {
    batt = 0;
    BattOut();
  };
  
  // dummy conversion (reset ADC)
  digitalWrite(DSET, HIGH);
  digitalWrite(DRESET, LOW); // L on CONV
  SPI.transfer16(0x0000);
  digitalWrite(DRESET, HIGH);

  wdt_disable();

  count++;            // Next integration
  TCNT1 = 0;          // reset Timer 1 counter
  uint16_t systime = TCNT3L; // read system time
  systime |= TCNT3H<<8;
  event_time[0] = systime;
}


void loop()
{
  for(int n=0; n<CHANNELS; n++) // reset histogram
  {
    histogram[n]=0;
  };
  events_counter = 1; // reset event counter

  // dummy conversion
  digitalWrite(DSET, HIGH);
  digitalWrite(DRESET, LOW);
  SPI.transfer16(0x0000);
  digitalWrite(DRESET, HIGH);

  {
    TCNT1 = 0;          // reset Timer 1 counter
    uint16_t systime = TCNT3L; // read system time
    systime |= TCNT3H<<8;
    event_time[0] = systime;
  }
  
  store = 0;
  batt = 0;
  env = 0;
  // dosimeter integration
  while(true)
  {
    while((PINB & 1)==0) // Waiting for signal drop
    {
      if (store >= 2) // Data out every 10 s
      {
        if (!digitalRead(BTN_USER_A))
        {
          playClickChangeTone();
          clicks = !clicks;
          powerOffByButton();
        };

        PostIntegration();
      }
    };
    // Signal is going down, we can run ADC
    // delayMicroseconds(4); // This delay is done in cycle overhead
    digitalWrite(DRESET, LOW); // L on CONV
    uint16_t adcVal = SPI.transfer16(0x0000); // 0c8000 +/GND, 0x0000 +/-

    if (clicks)
    {
      //if (adcVal>=CHANNELS) PORTD ^= 0x80; // digitalWrite(BUZZER, !digitalRead(BUZZER)); // buzzer click
      if (adcVal>=CHANNELS) PORTD ^= 0x20; // LED2 blick
      //if (adcVal>=CHANNELS) PORTD ^= 0xA0; // LED2 blick & BUZZER click
    };
    
    if (adcVal<CHANNELS)  // Record single event if energy is above threshold
    {   
      histogram[adcVal]++;
    }
    else
    {
//adcVal = 65535;
      if (events_counter<MAX_EVENTS) 
      {
        uint16_t reltime = TCNT1; //uint16_t reltime = TCNT1L; reltime |= TCNT1H<<8;
//reltime = 40000;
        event_time[events_counter] = reltime;
        event_time2[events_counter] = store;
        event_channel[events_counter] = adcVal;
      }
      events_counter++;
    }
    digitalWrite(DRESET, HIGH);
  }
}
