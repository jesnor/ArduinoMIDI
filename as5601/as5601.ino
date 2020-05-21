// 469-1070-ND funkar perfekt utan spacers, AGC runt 100
// 469-1025-ND funkar perfekt med 1 mm spacers, AGC runt 80
// 469-1075-ND funkar perfekt utan spacers, AGC runt 90

const int sdaPin = 10;
const int sclPin = 11;
const int address = 0x36;

void I2C_delay(void) {
//  delayMicroseconds(1);
}

void setInput(int pin) {
  pinMode(pin, /*pin == 20 || pin == 21 ? INPUT :*/ INPUT_PULLUP);   // Pins 20 and 21 have fixed 10k pullup resistors
}

int readPin(int pin) {
  setInput(pin);
  return digitalRead(pin) == LOW ? 0 : 1;  
}

void setPin(int pin) {
  setInput(pin);
}

void clearPin(int pin) {
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

int read_SDA() {
  return readPin(sdaPin);
}

void set_SDA() {
  setPin(sdaPin);
}

void clear_SDA() {
  clearPin(sdaPin);
/*  I2C_delay();
    
  if (digitalRead(sdaPin) == HIGH)
    error("SDA high!");*/
}

void write_SDA(int v) {
  if (v == 0)
    clear_SDA();
  else
    set_SDA();
}

int read_SCL() {
  return readPin(sclPin);
}

void set_SCL() {
  setPin(sclPin);
/*  I2C_delay();
    
  if (digitalRead(sclPin) == LOW)
    error("SCL low!");*/
}

void clear_SCL() {
  clearPin(sclPin);
/*  I2C_delay();
    
  if (digitalRead(sclPin) == HIGH)
    error("SCL high!");*/
}

void write_SCL(int v) {
  if (v == 0)
    clear_SCL();
  else
    set_SCL();
}

bool started = false;

void error(const char* text) {
  Serial.print("Error: ");
  Serial.println(text);
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

byte i2c_read_byte(int nack) {
  byte v = 0;

  for (int bit = 0; bit < 8; ++bit) {
    v = (v<< 1) | i2c_read_bit();
  }

  i2c_write_bit(nack);
  return v;
}

void setup() {
  Serial.begin(115200);  // start serial for output
  i2c_init();
}

void beginRead(int reg) {
  i2c_start();

  if (i2c_write_byte((address << 1) | 0) != 0)
    error("Got NACK!");
  
  if (i2c_write_byte(reg) != 0)
    error("Got NACK 2!");

  i2c_stop();
}

int endRead16() {
  i2c_start();

  if (i2c_write_byte((address << 1) | 1) != 0)
    error("Got NACK 16!");
    
  int msb = i2c_read_byte(0);
  int lsb = i2c_read_byte(1);
  i2c_stop();
  return word(msb, lsb);
}

byte read8(int reg) {
  beginRead(reg);
  i2c_start();

  if (i2c_write_byte((address << 1) | 1) != 0)
    error("Got NACK 8!");
    
  byte v = i2c_read_byte(1);
  i2c_stop();
  return v;
}

int lv = 0;
bool initReg = true;

void loop() {
  unsigned long t = micros();
  
  if (initReg) {
    beginRead(12);
//    initReg = false;
  }
  
  int v = endRead16();
  unsigned long dt = micros() - t;

  if (lv != v) {
    //int u = read16(14) >> 3;
    int agc = read8(0x1A);
   
    Serial.print(lv);
    Serial.print(" ");
    Serial.print(v);
    Serial.print(" ");
    Serial.print(dt);
    Serial.print(" ");
    Serial.print(agc);
/*    Serial.print(", angle: ");
    Serial.print(u);
    Serial.print(", AGC: ");
    Serial.print(agc);
    Serial.print(", time: ");
    Serial.print(dt);
    Serial.print(" us");*/
    Serial.println();
    lv = v;
  }

  dt = micros() - t;
  unsigned long d = 10000 - dt;

  if (d > 0 && d < 10000)
    delayMicroseconds(d);
}
