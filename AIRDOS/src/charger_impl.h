
#define CHARGER_ADDR 0x6A

uint8_t readChargerReg(uint8_t regaddr)
{
  Wire.beginTransmission(CHARGER_ADDR);
  Wire.write(regaddr);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)CHARGER_ADDR, (uint8_t)1);
  if (!Wire.available()) return 0;
  return (uint8_t)Wire.read();
}

uint16_t readChargerReg16(uint8_t regaddr)
{
  Wire.beginTransmission(CHARGER_ADDR);
  Wire.write(regaddr);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)CHARGER_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;

  uint16_t lsb = (uint16_t)Wire.read();  // první byte = LSB
  uint16_t msb = (uint16_t)Wire.read();  // druhý byte = MSB

  return (msb << 8) | lsb;               // složit správně
}

void writeChargerReg(uint8_t regaddr, uint8_t value)
{
  Wire.beginTransmission(CHARGER_ADDR);
  Wire.write(regaddr);
  Wire.write(value);
  Wire.endTransmission();
}

// Single charger configuration function: enable or disable charging
void configCharger(bool EnCharging)
{
  // Input current limit register (same in both modes)
  Wire.beginTransmission(CHARGER_ADDR);
  Wire.write((uint8_t)0x02); // Input current limit register
  Wire.write((uint8_t)(int(440/40))<<5); // 440 mA
  Wire.endTransmission();

  // Charge control block: different payload depending on desired state
  Wire.beginTransmission(CHARGER_ADDR);
  Wire.write((uint8_t)0x14); // Charge control block
  if (EnCharging)
  {
    Wire.write((uint8_t)0b00100110);
    Wire.write((uint8_t)0b00011001);
    Wire.write((uint8_t)0b10100000); // Enable charger
    Wire.write((uint8_t)0b01010110);
    Wire.write((uint8_t)0b00000000);
    Wire.write((uint8_t)0b00000001);
  }
  else
  {
    Wire.write((uint8_t)0b00100110);
    Wire.write((uint8_t)0b10011001);
    Wire.write((uint8_t)0b00000000); // Disable charger
    Wire.write((uint8_t)0b01010110);
    Wire.write((uint8_t)0b00000000);
    Wire.write((uint8_t)0b00000001);
  }
  Wire.endTransmission();
  // NTC configuration (same in both modes)
  Wire.beginTransmission(CHARGER_ADDR); // NTC
  Wire.write((uint8_t)0x1a);
  Wire.write((uint8_t)0b10111111);
  Wire.endTransmission();

  // ADC configuration (same in both modes)
  Wire.beginTransmission(CHARGER_ADDR); // ADC configuration
  Wire.write((uint8_t)0x26);
  Wire.write((uint8_t)0b10001100);
  Wire.endTransmission();
}

bool detectBatteryPresence(uint16_t &batteryMv)
{
  // Step 1: Disable charging (EN_CHG = 0)
  uint8_t reg16 = readChargerReg(0x16);
  reg16 &= ~(1u << 5);
  writeChargerReg(0x16, reg16);
  delay(50);

  // Step 2: vynutit vybíjecí proud IBAT (FORCE_IBATDIS = 1) 
  reg16 |= (1u << 6);                      // bit6 = FORCE_IBATDIS
  writeChargerReg(0x16, reg16);
  delay(200);                               // krátký vybíjecí „pulz“

  // Step 3: zrušit vybíjení IBAT (FORCE_IBATDIS = 0) 
  reg16 &= ~(1u << 6);
  writeChargerReg(0x16, reg16);
  delay(20);                               // mezera po pulzu

  // Step 4: povolit VBAT ADC a nastavit one-shot 
  // (jen pro jistotu zajistíme, že VBAT ADC není zakázaný v REG0x27)
  uint8_t reg27 = readChargerReg(0x27);    // ADC_FUNCTION_DISABLE_0
  reg27 &= ~(1u << 4);                     // bit4 = VBAT_ADC_DIS -> 0 = enable
  writeChargerReg(0x27, reg27);

  uint8_t reg26 = readChargerReg(0x26);    // ADC_CONTROL
  reg26 |= (1u << 7);                      // bit7 = ADC_EN
  reg26 |= (1u << 6);                      // bit6 = ADC_RATE = 1 (one-shot)
  reg26 &= ~(3u << 4);    // ADC_SAMPLE = 00
  reg26 |= (1u << 1);     // ADC_AVG_INIT = 1
  writeChargerReg(0x26, reg26);

  // dle datasheetu ~30 ms pro prevod ADC
  delay(50);

  //  Step 5: přečíst VBAT_ADC (REG0x30, bity 12:1) 
  uint16_t vbatRaw  = readChargerReg16(0x30);
  uint16_t vbatCode = (vbatRaw >> 1) & 0x0FFF;   // bity 12:1, bit0 je reserved

  uint32_t tmp = (uint32_t)vbatCode * VBAT_ADC_LSB_0P01_MV;
  batteryMv = (uint16_t)(tmp / 100u);               // mV

  // //  Debug výpis 
  // Serial1.print("#BATDEBUG raw=0x");
  // Serial1.print(vbatRaw, HEX);
  // Serial1.print(" bin=0b");
  // Serial1.print(vbatRaw, BIN);
  // Serial1.print(" code=");
  // Serial1.print(vbatCode);
  // Serial1.print(" mv=");
  // Serial1.print(batteryMv);

  bool present = (batteryMv >= VBAT_PRESENT_THRESHOLD_MV);

  return present;
}

uint8_t getChargerVBUSVoltage()
{
  Wire.beginTransmission(CHARGER_ADDR);      // ADC of VBUS
  Wire.write(0x2D); // MSB 0.264 V/bit
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)CHARGER_ADDR, (uint8_t)1);
  uint8_t vbus = Wire.read() & 0x7F;
  return vbus;
}


int16_t readBat(int8_t regaddr)
{
  Wire.beginTransmission(BQ34Z100);
  Wire.write(regaddr);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100,1);

  unsigned int low = Wire.read();

  Wire.beginTransmission(BQ34Z100);
  Wire.write(regaddr+1);
  Wire.endTransmission();

  Wire.requestFrom(BQ34Z100,1);

  unsigned int high = Wire.read();

  unsigned int high1 = high<<8;

  return (high1 + low);
}

