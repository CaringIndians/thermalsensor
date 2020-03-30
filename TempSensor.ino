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

#define AVERAGING_WINDOW 10

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

// lcd constants
#define LCD_BRIGHTNESS_HIGH 1
#define LCD_BRIGHTNESS_LOW 0
#define LCD_TEMP_PRECISION 2

// lcd pins
#define LCD_RS_PIN 10
#define LCD_EN_PIN 9
#define LCD_D4_PIN 7
#define LCD_D5_PIN 6
#define LCD_D6_PIN 5
#define LCD_D7_PIN 4
#define LCD_BL_PIN 3
#define LCD_BR_PIN 11

// trigger pins
#define TRIGGER_PIN 2

// laser pins
#define LASER_PIN 8

// operational settings
#define REFRESH_TEMP_READING 200

// feature flags
#define FLAG_MOVING_AVG_ENABLE false

class LiquidCrystalBacklight{
  private:
    uint8_t _backlight_pin;
    uint8_t _brightness_pin;
  public:
    LiquidCrystalBacklight(uint8_t bl, uint8_t br)
    {
      _backlight_pin = bl;
      _brightness_pin = br;
    }

    void display(){
      digitalWrite(_backlight_pin, HIGH);
      analogWrite(_brightness_pin, LCD_BRIGHTNESS_HIGH);
    }
    void noDisplay(){
      digitalWrite(_backlight_pin, LOW);
      analogWrite(_brightness_pin, LCD_BRIGHTNESS_LOW);
    }
};

class Trigger {
  public:
    void begin(){
      pinMode(TRIGGER_PIN, INPUT);
      digitalWrite(TRIGGER_PIN, HIGH);
    }
    bool readTriggerStatus(){
      return digitalRead(TRIGGER_PIN);
    }
};

class Laser {
  public:
    void begin(){
      pinMode(LASER_PIN, OUTPUT);
    }
    void on(){
      digitalWrite(LASER_PIN, HIGH);
    }
    void off(){
      digitalWrite(LASER_PIN, LOW);
    }
};

// initialize trigger
Trigger trigger;

// initialize laser
Laser laser;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
LiquidCrystal lcd(
  LCD_RS_PIN,
  LCD_EN_PIN,
  LCD_D4_PIN,
  LCD_D5_PIN,
  LCD_D6_PIN,
  LCD_D7_PIN
);
LiquidCrystalBacklight lcd_backlight(
  LCD_BL_PIN,
  LCD_BR_PIN
);

class RotatingCounter {
  private:
  	int m_counter = 0;
    bool m_first_pass = true;
  	int m_max_value = 0;
  public:
    void setMaxValue(int max_value) {
      m_max_value = max_value;
    }
  	void resetCounter(){
      m_counter = 0;
      m_first_pass = true;
    }
    void increment(){
      m_counter++;
      if (m_counter >= m_max_value){
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
    double m_sum = 0;
    float m_values[AVERAGING_WINDOW];
    RotatingCounter m_rotating_counter;
  public:
    MovingAverage() {
      m_rotating_counter.setMaxValue(AVERAGING_WINDOW);
    }
    float getAverage(){
      if (m_rotating_counter.isFirstPass()) {
        return m_sum/AVERAGING_WINDOW;
      }
      else {
        int count = m_rotating_counter.getCounterValue();
        if (count == 0) {
          return 0;
        }
        return m_sum/count;
      }
    }
    void pushValue(float value){
      int pos = m_rotating_counter.getCounterValue();
      m_sum = m_sum - m_values[pos];
      m_values[pos] = value;
      m_sum = m_sum + value;
      m_rotating_counter.increment();
    }
    void reset(){
      m_rotating_counter.resetCounter();
    }
};

void setup() {
  // set up trigger button
  trigger.begin();
  // set up laser
  laser.begin();
  // set up IR sensor
  mlx.begin();
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2); 
  lcd.home();
}


MovingAverage avg_celsius;
MovingAverage avg_fahrenheit;

void loop() {
  float temp_celcius;
  float temp_fahrenheit;

  switch (trigger.readTriggerStatus()){
    case LOW:
      // Start laser
      laser.on();

      // Get temperature readings
      temp_celcius = mlx.readObjectTempC();
      temp_fahrenheit = mlx.readObjectTempF();
      // Populate temperature values for moving average
      if (FLAG_MOVING_AVG_ENABLE) {
        avg_celsius.pushValue(temp_celcius);
        avg_fahrenheit.pushValue(temp_fahrenheit);
      }

      lcd.setCursor(0,0);
      if (FLAG_MOVING_AVG_ENABLE) {
        lcd.print(avg_celsius.getAverage(), LCD_TEMP_PRECISION);
      }
      else {
        lcd.print(temp_celcius, LCD_TEMP_PRECISION);
      }
      lcd.print(" C");
      lcd.setCursor(0,1);
      if (FLAG_MOVING_AVG_ENABLE) {
        lcd.print(avg_fahrenheit.getAverage(), LCD_TEMP_PRECISION);
      }
      else{
        lcd.print(temp_fahrenheit, LCD_TEMP_PRECISION);
      }
      lcd.print(" F");
      lcd_backlight.display();
      lcd.display();
      break;
    case HIGH:
      laser.off();
      if (FLAG_MOVING_AVG_ENABLE) {
        avg_celsius.reset();
        avg_fahrenheit.reset();
      }
      lcd_backlight.noDisplay();
      lcd.noDisplay();
      break;
  }
  delay(REFRESH_TEMP_READING);
}