/*
 TempSensor is an Arduino based software for reading temperatures
 from MLX90614 IR thermopile. 

 We use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. 

 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

*/

// include the library code:
#include <LiquidCrystal.h>

// taken from https://github.com/adafruit/Adafruit-MLX90614-Library
// TODO (krishnadurai): Comply with license
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"


#define MLX90614_I2CADDR 0x5A

// RAM
#define MLX90614_RAWIR1 0x04
#define MLX90614_RAWIR2 0x05
#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07
#define MLX90614_TOBJ2 0x08
// EEPROM
#define MLX90614_TOMAX 0x20
#define MLX90614_TOMIN 0x21
#define MLX90614_PWMCTRL 0x22
#define MLX90614_TARANGE 0x23
#define MLX90614_EMISS 0x24
#define MLX90614_CONFIG 0x25
#define MLX90614_ADDR 0x0E
#define MLX90614_ID1 0x3C
#define MLX90614_ID2 0x3D
#define MLX90614_ID3 0x3E
#define MLX90614_ID4 0x3F


class MLX90614  {
 public:
  MLX90614(uint8_t addr = MLX90614_I2CADDR);
  boolean begin();
  uint32_t readID(void);

  double readObjectTempC(void);
  double readAmbientTempC(void);
  double readObjectTempF(void);
  double readAmbientTempF(void);

 private:
  float readTemp(uint8_t reg);

  uint8_t _addr;
  uint16_t read16(uint8_t addr);
  void write16(uint8_t addr, uint16_t data);
};


MLX90614::MLX90614(uint8_t i2caddr) {
  _addr = i2caddr;
}


boolean MLX90614::begin(void) {
  Wire.begin();

  /*
  for (uint8_t i=0; i<0x20; i++) {
    Serial.print(i); Serial.print(" = ");
    Serial.println(read16(i), HEX);
  }
  */
  return true;
}

//////////////////////////////////////////////////////


double MLX90614::readObjectTempF(void) {
  return (readTemp(MLX90614_TOBJ1) * 9 / 5) + 32;
}


double MLX90614::readAmbientTempF(void) {
  return (readTemp(MLX90614_TA) * 9 / 5) + 32;
}

double MLX90614::readObjectTempC(void) {
  return readTemp(MLX90614_TOBJ1);
}


double MLX90614::readAmbientTempC(void) {
  return readTemp(MLX90614_TA);
}

float MLX90614::readTemp(uint8_t reg) {
  float temp;
  
  temp = read16(reg);
  temp *= .02;
  temp  -= 273.15;
  return temp;
}

/*********************************************************************/

uint16_t MLX90614::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(_addr); // start transmission to device 
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(false); // end transmission
  
  Wire.requestFrom(_addr, (uint8_t)3);// send data n-bytes read
  ret = Wire.read(); // receive DATA
  ret |= Wire.read() << 8; // receive DATA

  uint8_t pec = Wire.read();

  return ret;
}

MLX90614 mlx = MLX90614();

const int refreshTempReading = 500;
const int averagingWindow = 10;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

class RotatingCounter {
  private:
  	int m_counter = 0;
    bool m_first_pass = true;
  	int m_max_value = 0;
  public:
  	RotatingCounter(int max_value) {
      m_max_value = max_value;
    }
  	void resetCounter(){
      m_counter = 0;
      m_first_pass = true;
    }
    void increment(){
      m_counter++;
      if (m_counter > m_max_value){
        m_counter = 0;
        m_first_pass = false;
      }
    }
    int isFirstPass() {
      return m_first_pass;
    }
    int getCounterValue() {
      return m_counter;
    }
};

class MovingAverage {
  private:
  	int * m_values;
    long m_sum = 0;
    RotatingCounter * m_rotating_counter;
  public:
    MovingAverage() {
      m_values = new int[averagingWindow];
      m_rotating_counter = new RotatingCounter(10);
    }
    int getAverage(){
      if (m_rotating_counter->isFirstPass() == false) {
        return m_sum/averagingWindow;
       }
      else {
        return m_sum/m_rotating_counter->getCounterValue();
      }
    }
    void pushValue(int value){
      int pos = m_rotating_counter->getCounterValue();
      m_sum = m_sum - m_values[pos];
      m_values[pos] = value;
      m_sum = m_sum + value;
      m_rotating_counter->increment();
    }
};

void setup() {
  mlx.begin();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2); 
  lcd.home ();
}


// TODO(krishnadurai): Temperature readings and MovingAverages
// are typed in `int`. We may need to move to a `float` based type.
void loop() {
  MovingAverage avg_celsius;
  MovingAverage avg_fahrenheit;
  
  // Get temperature readings
  int temp_celcius = mlx.readObjectTempC();
  int temp_fahrenheit = mlx.readObjectTempF();

  // Populate temperature values for moving average
  avg_celsius.pushValue(temp_celcius);
  avg_fahrenheit.pushValue(temp_fahrenheit);

  // Turn off the display:
  lcd.noDisplay();
  delay(refreshTempReading);

  // Turn on the display:
  lcd.setCursor(0,0);
  lcd.print(avg_celsius.getAverage());
  lcd.print(" C");
  lcd.setCursor(0,1);
  lcd.print(avg_fahrenheit.getAverage());
  lcd.print(" F");
  lcd.display();
  delay(refreshTempReading);
}