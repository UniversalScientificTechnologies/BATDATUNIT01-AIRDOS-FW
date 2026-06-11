 


// Read EEPROM structure from internal EEPROM
bool readEEPROMRecord(uint8_t eeprom_addr, eeprom::EepromRecord &record)
{
  Wire.beginTransmission(eeprom_addr);
  Wire.write((uint8_t)0x00); // MSB - start from address 0
  Wire.write((uint8_t)0x00); // LSB
  Wire.endTransmission();
  
  uint8_t* ptr = (uint8_t*)&record;
  uint8_t bytesRead = 0;
  
  // Read in chunks due to Wire buffer limitations
  while (bytesRead < sizeof(eeprom::EepromRecord))
  {
    uint8_t toRead = min(sizeof(eeprom::EepromRecord) - bytesRead, 16);
    Wire.requestFrom(eeprom_addr, toRead);
    
    while (Wire.available() && bytesRead < sizeof(eeprom::EepromRecord))
    {
      ptr[bytesRead++] = Wire.read();
    }
    
    if (bytesRead < sizeof(eeprom::EepromRecord))
    {
      // Set address for next chunk
      Wire.beginTransmission(eeprom_addr);
      Wire.write((uint8_t)(bytesRead >> 8));   // MSB
      Wire.write((uint8_t)(bytesRead & 0xFF)); // LSB
      Wire.endTransmission();
    }
  }
  
  return (bytesRead == sizeof(eeprom::EepromRecord));
}

void readEepromSNToString(uint8_t eeprom_addr, String& dataString)
{
  Wire.beginTransmission(eeprom_addr);                   // request SN from EEPROM
  Wire.write((int)0x08); // MSB
  Wire.write((int)0x00); // LSB
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)eeprom_addr, 16);
  for (int8_t reg=0; reg<16; reg++)
  {
    uint8_t serialbyte = Wire.read(); // receive a byte
    if (serialbyte<0x10) dataString += "0";
    dataString += String(serialbyte,HEX);
  }
}
