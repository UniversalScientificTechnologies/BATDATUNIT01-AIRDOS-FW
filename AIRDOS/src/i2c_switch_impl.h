#define PCA9541_ADDR  0x70      
#define REG_CONTROL   0x01
#define REG_ISTAT     0x02

#define CTL_MYBUS   (1<<0)
#define CTL_NMYBUS  (1<<1)
#define CTL_BUSON   (1<<2)
#define CTL_NBUSON  (1<<3)
#define CTL_BUSINIT (1<<4)

#define MYBUS_PAIR  (CTL_MYBUS | CTL_NMYBUS)   // 0x03
#define BUSON_PAIR  (CTL_BUSON | CTL_NBUSON)   // 0x0C

static inline bool isMine(uint8_t r)  
{ 
  return ((r & MYBUS_PAIR) == 0) || ((r & MYBUS_PAIR) == MYBUS_PAIR); // bity shodné = moje
}

static inline bool isBusOn(uint8_t r) 
{ 
  return ((r & BUSON_PAIR) != 0) && ((r & BUSON_PAIR) != BUSON_PAIR); // bity různé = ON
}   

uint8_t pca_readCtrl() {
  Wire.beginTransmission(PCA9541_ADDR);
  Wire.write(REG_CONTROL);
  Wire.endTransmission(false);                 // repeated start
  Wire.requestFrom((uint8_t)PCA9541_ADDR, (uint8_t)1);
  return Wire.read();
}

void pca_writeCtrl(uint8_t v) {
  Wire.beginTransmission(PCA9541_ADDR);
  Wire.write(REG_CONTROL);
  Wire.write(v);
  Wire.endTransmission();
}

bool i2c_switch_forceTakeBus() 
{
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t r = pca_readCtrl();

    if (isMine(r) && isBusOn(r)) {
      pca_writeCtrl(r | CTL_BUSINIT);   // mám ji → pro jistotu pročisti downstream
      delay(2);                          // ať se 9 hodin + STOP stihne dokončit
      return true;
    }

    uint8_t keep = r & 0xC0;             // zachovej TESTON/NTESTON
    pca_writeCtrl(!isMine(r) ? (keep | MYBUS_PAIR)                 // nárokuj (odpojí druhého)
                             : (keep | MYBUS_PAIR | BUSON_PAIR));  // moje, ale OFF → připoj
    delay(1);
  }
  return false;
}

bool i2c_switch_isBusMine() 
{
  uint8_t r = pca_readCtrl();
  return isMine(r) && isBusOn(r);
}


