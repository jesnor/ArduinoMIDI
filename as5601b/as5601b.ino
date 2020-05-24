// 469-1070-ND funkar perfekt utan spacers, AGC runt 100
// 469-1025-ND funkar perfekt med 1 mm spacers, AGC runt 80
// 469-1075-ND funkar perfekt utan spacers, AGC runt 90

int cMask = 0xC0;
int fMask = 0xF3;
int bMask = 0x70;
int dMask = 0xF2;
int eMask = 0x4;

const int i2cAddress = 0x36;

void I2C_delay(void) {
  delayMicroseconds(100);
}

void setInputMode() {
  DDRC = 0;
  DDRF = 0;
  DDRB = 0;
  DDRD = 0;  
}

void setOutputMode() {
  DDRC = cMask;
  DDRF = fMask;
  DDRB = bMask;
  DDRD = dMask;  
}

void setOutputs() {
  PORTC = cMask;
  PORTF = fMask;
  PORTB = bMask;
  PORTD = dMask;
}

void clearOutputs() {
  PORTC = 0;
  PORTF = 0;
  PORTB = 0;
  PORTD = 0;
}

int read_SDA() {
  setInputMode();
  setOutputs();

  // Read data
  int c = PINC;
  int f = PINF;
  int b = PINB;
  int d = PIND;

  return 
    ((c >> 7) & 1) |
    (((c >> 6) & 1) << 1) |
    (((f >> 7) & 1) << 2) |
    (((f >> 6) & 1) << 3) |
    
    (((f >> 5) & 1) << 4) |
    (((f >> 4) & 1) << 5) |
    (((f >> 1) & 1) << 6) |
    ((f & 1) << 7) |
    
    (((b >> 6) & 1) << 8) |
    (((b >> 5) & 1) << 9) |
    (((b >> 4) & 1) << 10) |
    (((d >> 7) & 1) << 11) |

    (((d >> 6) & 1) << 12) |
    (((d >> 4) & 1) << 13) |
    (((d >> 1) & 1) << 14) |
    (((d >> 5) & 1) << 15);
}

void set_SDA() {
  setOutputs();
  setOutputMode();
}

void clear_SDA() {
  clearOutputs();
  setOutputMode();

  /*  I2C_delay();

    if (myDigitalRead(sdaPin) == HIGH)
    error("SDA high!");*/
}

void write_SDA(int v) {
  if (v == 0)
    clear_SDA();
  else
    set_SDA();
}

//int read_SCL() {
//  return ((PINE >> 2) & 1) == 1;
//}

void set_SCL() {
  PORTE = eMask;
  DDRE = eMask;
  /*  I2C_delay();

    if (myDigitalRead(sclPin) == LOW)
      error("SCL low!");*/
}

void clear_SCL() {
  PORTE = 0;
  DDRE = eMask;
  /*  I2C_delay();

    if (myDigitalRead(sclPin) == HIGH)
      error("SCL high!");*/
}

void write_SCL(int v) {
  if (v == 0)
    clear_SCL();
  else
    set_SCL();
}

bool started = false;

void error(const char* text, int v) {
  Serial.print("Error: ");
  Serial.print(text);
  Serial.print(", ");
  Serial.println(v, HEX);
}

void i2c_init() {
  set_SDA();
  set_SCL();
  I2C_delay();
}

void i2c_start() {
  clear_SDA();
  I2C_delay();
  clear_SCL();
  I2C_delay();
}

void i2c_stop() {
  clear_SDA();
  I2C_delay();
  set_SCL();
  I2C_delay();
  set_SDA();
  I2C_delay();
}

void i2c_write_bit(int bit) {
  write_SDA(bit);
  I2C_delay();
  set_SCL();
  I2C_delay();
  clear_SCL();
}

int i2c_read_bit() {
  set_SDA();
  I2C_delay();
  set_SCL();
  I2C_delay();
  int bit = read_SDA();
  clear_SCL();
  //I2C_delay();
  return bit;
}

int i2c_write_byte(byte v) {
  for (int bit = 0; bit < 8; ++bit) {
    i2c_write_bit((v & 0x80) != 0);
    v <<= 1;
  }

  return i2c_read_bit();
}

void i2c_read_byte(int nack, byte* values) {
  for (int i = 0; i < 16; i++)
    values[i] = 0;

  for (int bit = 0; bit < 8; ++bit) {
    int bits = i2c_read_bit();

    for (int i = 0; i < 16; i++)
      values[i] = (values[i] << 1) | ((bits >> i) & 1);
  }

  i2c_write_bit(nack);
}

void setup() {
  Serial.begin(115200);  // start serial for output
  i2c_init();
}

void beginRead(int reg) {
  i2c_start();
  int v = i2c_write_byte(i2cAddress << 1);

  if (v != 0)
    error("Got NACK!", v);

  int v2 = i2c_write_byte(reg);

  if (v2 != 0)
    error("Got NACK 2!", v2);

  i2c_stop();
}

void endRead16(int* values) {
  i2c_start();
  int v = i2c_write_byte((i2cAddress << 1) | 1);
  
  if (v != 0)
    error("Got NACK 16!", v);

  byte msbs[16];
  byte lsbs[16];
  i2c_read_byte(0, msbs);
  i2c_read_byte(1, lsbs);
  i2c_stop();

  for (int i=0; i<16; i++)
    values[i] = word(msbs[i], lsbs[i]);
}

void read8(int reg, byte* values) {
  beginRead(reg);
  i2c_start();
  int v = i2c_write_byte((i2cAddress << 1) | 1);

  if (v != 0)
    error("Got NACK 8!", v);

  i2c_read_byte(1, values);
  i2c_stop();
}

int lv = 0;
bool initReg = true;
int angles[16];
int lastAngles[16];
byte agcs[16];

void loop() {
  unsigned long t = micros();

  if (initReg) {
    beginRead(12);
    //    initReg = false;
  }

  endRead16(angles);
  read8(0x1A, agcs);
  //int u = read16(14) >> 3;
  unsigned long dt = micros() - t;

  for (int i=0; i<16; i++) {
    if (agcs[i] < 255 && angles[i] != lastAngles[i]) {
      Serial.print(i);
      Serial.print(": ");
      Serial.print(lastAngles[i]);
      Serial.print(" ");
      Serial.print(angles[i]);
      Serial.print(" ");
      Serial.print(dt);
      Serial.print(" ");
      Serial.print(agcs[i]);
      /*    Serial.print(", angle: ");
          Serial.print(u);
          Serial.print(", AGC: ");
          Serial.print(agc);
          Serial.print(", time: ");
          Serial.print(dt);
          Serial.print(" us");*/
      Serial.println();
      lastAngles[i] = angles[i];
    }
  }

  dt = micros() - t;
  unsigned long d = 10000 - dt;

  if (d > 0 && d < 10000)
    delayMicroseconds(d);
}
