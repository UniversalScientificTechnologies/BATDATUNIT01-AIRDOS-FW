
#define RTC_ADDR 0x51

uint8_t rtc_current_time_s100; //.00 part of RTC time
uint32_t rtc_current_time_s;  // Current RTC time in seconds
uint32_t eeprom_sync_time;  // Last sync time from EEPROM
uint32_t eeprom_sync_rtc_seconds; //how many second was on rtc timer when eeprom sync was done
 


uint8_t bcdToDec(uint8_t b)
{
  return ( ((b >> 4)*10) + (b%16) );
}

inline uint32_t getCurrentUnixTime()
{
  return rtc_current_time_s+eeprom_sync_time-eeprom_sync_rtc_seconds;
}

void readRTC()
{
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(0);
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)6);
  rtc_current_time_s100 = bcdToDec(Wire.read()); //setiny
  uint8_t tm_sec = bcdToDec(Wire.read() & 0x7f); //sekundy
  uint8_t tm_min = bcdToDec(Wire.read() & 0x7f); //minuty
  rtc_current_time_s = bcdToDec(Wire.read()); //hodiny
  rtc_current_time_s += bcdToDec(Wire.read()) * 100UL; //stovky hodin
  rtc_current_time_s += bcdToDec(Wire.read()) * 10000UL; //desetitisíce hodin
  rtc_current_time_s = rtc_current_time_s * 60 * 60 + tm_min * 60 + tm_sec;
}

// Read single register from RTC (PCF? at address RTC_ADDR)
uint8_t readRTCreg(uint8_t reg)
{
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)RTC_ADDR, (uint8_t)1);
  if (Wire.available()) return (uint8_t)Wire.read();
  return 0xFF; // error
}

// Write single register to RTC
void writeRTCreg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(RTC_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Reset RTC into stopwatch mode using the provided initialization sequence
// and append a log entry to SD card (assumes SD is already initialized and `filename` is set).
void ResetRTC()
{
  // Use the exact sequence provided in AIRDOS04X for initialization
  Wire.beginTransmission(RTC_ADDR); // init clock
  Wire.write((uint8_t)0x23); // Start register
  Wire.write((uint8_t)0x00); // 0x23
  Wire.write((uint8_t)0x00); // 0x24 Two's complement offset value
  Wire.write((uint8_t)0b00000101); // 0x25 Normal offset correction, disable low-jitter mode, set load caps to 6 pF
  Wire.write((uint8_t)0x00); // 0x26 Battery switch reg, same as after a reset
  Wire.write((uint8_t)0x00); // 0x27 Enable CLK pin, using bits set in reg 0x28
  Wire.write((uint8_t)0x97); // 0x28 stop watch mode, no periodic interrupts, CLK pin off
  Wire.write((uint8_t)0x00); // 0x29
  Wire.write((uint8_t)0x00); // 0x2a
  Wire.endTransmission();

  Wire.beginTransmission(RTC_ADDR); // reset clock
  Wire.write(0x2f);
  Wire.write(0x2c);
  Wire.endTransmission();

  Wire.beginTransmission(RTC_ADDR); // start stop-watch
  Wire.write(0x28);
  Wire.write(0x97);
  Wire.endTransmission();

  Wire.beginTransmission(RTC_ADDR); // reset stop-watch
  Wire.write((uint8_t)0x00); // Start register
  Wire.write((uint8_t)0x00); // 0x00
  Wire.write((uint8_t)0x00); // 0x01
  Wire.write((uint8_t)0x00); // 0x02
  Wire.write((uint8_t)0x00); // 0x03
  Wire.write((uint8_t)0x00); // 0x04
  Wire.write((uint8_t)0x00); // 0x05
  Wire.endTransmission();
}


// Convert Unix timestamp to human readable format (UTC)
void unixToDateTime(uint32_t unix_time, uint16_t &year, uint8_t &month, uint8_t &day, 
                    uint8_t &hour, uint8_t &minute, uint8_t &second)
{
  second = unix_time % 60;
  unix_time /= 60;
  minute = unix_time % 60;
  unix_time /= 60;
  hour = unix_time % 24;
  unix_time /= 24;
  
  uint16_t days = unix_time;
  year = 1970;
  
  while (true)
  {
    uint16_t days_in_year = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) ? 366 : 365;
    if (days < days_in_year) break;
    days -= days_in_year;
    year++;
  }
  
  uint8_t days_in_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) days_in_month[1] = 29;
  
  month = 1;
  while (days >= days_in_month[month - 1])
  {
    days -= days_in_month[month - 1];
    month++;
  }
  day = days + 1;
}

void addHumanReadebleFromUnixTime(uint32_t unixTime, String &s)
{
  uint16_t year;
  uint8_t month, day, hour, minute, second;  
  unixToDateTime(unixTime, year, month, day, hour, minute, second);

  s += String(year);
  s += "-";
  if (month < 10) s += "0";
  s += String(month);
  s += "-";
  if (day < 10) s += "0";
  s += String(day);
  s += " ";
  if (hour < 10) s += "0";
  s += String(hour);
  s += ":";
  if (minute < 10) s += "0";
  s += String(minute);
  s += ":";
  if (second < 10) s += "0";
  s += String(second);
}

