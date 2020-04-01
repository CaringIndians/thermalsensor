/*
 Thermal Sensor is an Arduino based software for reading temperatures
 from MLX90614 IR thermopile. 

 We use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. 

 We use MLX90614 as an IR based thermopile sensor.

*/
/***********************************************************************************************//**
 *  \brief      Thermal Sensor device - CPP Source file
 *  \par
 *  \par        Details
 *              Thermal Sensor is an Arduino based software for reading temperatures
 *              from MLX90614 IR thermopile.

 *              We use a 16x2 LCD display.  The LiquidCrystal
 *              library works with all LCD displays that are compatible with the
 *              Hitachi HD44780 driver.

 *              We use MLX90614 as an IR based thermopile sensor.
 *
 *  \li         Sleep mode is not implemented yet.
 *
 *  \note       THIS IS ONLY A PARTIAL RELEASE. THIS DEVICE IS CURRENTLY UNDERGOING
 *              ACTIVE DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP
 *              THIS IN MIND IF YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.
 *
 *  \file       TempSensor.ino
 *  \author     Krishna Durai <krishnadurai20@gmail.com>
 *  \version    0.1
 *  \date       2020
 *  \copyright  Copyright &copy; 2020 Krishna Durai.  All right reserved.
 *
 *  \par        License
 *              This program is free software; you can redistribute it and/or modify it under
 *              the terms of the GNU Lesser General Public License as published by the Free
 *              Software Foundation; either version 2.1 of the License, or (at your option)
 *              any later version.
 *  \par
 *              This Program is distributed in the hope that it will be useful, but WITHOUT ANY
 *              WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *              PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details
 *              at http://www.gnu.org/copyleft/gpl.html
 *  \par
 *              You should have received a copy of the GNU Lesser General Public License along
 *              with this library; if not, write to the Free Software Foundation, Inc.,
 *              51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *//***********************************************************************************************/


// include the library code:
#include <LiquidCrystal.h>
#include <math.h>

#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#define CRC8_DEFAULTPOLY  7

class CRC8 {
  public:
    CRC8(uint8_t polynomial = CRC8_DEFAULTPOLY);
    uint8_t  crc8(void);
    uint8_t  crc8(uint8_t data);
    void     crc8Start(uint8_t poly);
  private:
    uint8_t  _crc;
    uint8_t  _poly;
};
/**
 *  \brief            CRC8 class constructor.
 *  \param [in] poly  8 bit CRC polynomial to use.
 */
CRC8::CRC8(uint8_t poly) {crc8Start(poly);}

/**
 *  \brief            Return the current value of the CRC.
 *  \return           8 bit CRC current value.
 */
uint8_t CRC8::crc8(void) {return _crc;}

/**
 *  \brief            Update the current value of the CRC.
 *  \param [in] data  New 8 bit data to be added to the CRC.
 *  \return           8 bit CRC current value.
 */
uint8_t CRC8::crc8(uint8_t data) {
    uint8_t i = 8;

    _crc ^= data;
    while(i--) _crc = _crc & 0x80 ? (_crc << 1) ^ _poly : _crc << 1;
    return _crc;
}

/**
 *  \brief            Initialize the CRC8 object.
 *  \param [in] poly  8 bit CRC polynomial to use.
 */
void CRC8::crc8Start(uint8_t poly) {
    _poly = poly;
    _crc = 0;
}

/***************************************************
  This is a library for the MLX90614 Temp Sensor
  Designed specifically to work with the MLX90614 sensors in the
  adafruit shop
  ----> https://www.adafruit.com/products/1748
  ----> https://www.adafruit.com/products/1749
  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
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
// CRC
#define MLX90614_CRC8POLY       7       /**< CRC polynomial = X8+X2+X1+1 */
// Write Delay
#define MLX90614_WRITE_DELAY 10

class MLX90614  {
 public:
  MLX90614(uint8_t addr = MLX90614_I2CADDR);
  boolean begin();
  uint32_t readID(void);

  double readObjectTempC(void);
  double readAmbientTempC(void);
  double readObjectTempF(void);
  double readAmbientTempF(void);
  void setEmissivity(float);
  float getEmissivity(void);

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

void MLX90614::setEmissivity(float emissivity) {
  uint16_t rawOld = read16(MLX90614_EMISS);
  uint16_t rawNew = round(emissivity * 65535);
  if (rawOld != rawNew) {
    write16(MLX90614_EMISS, 0);
    write16(MLX90614_EMISS, rawNew);
  }
};

float MLX90614::getEmissivity() {
  return ((float)read16(MLX90614_EMISS)) / 65535.0;
};

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

/**
 *  \brief            Write a 16 bit value to memory.
 *  \param [in] cmd   Command to send (register to write to).
 *  \param [in] data  Value to write.
 */
void MLX90614::write16(uint8_t cmd, uint16_t data) {
  CRC8 crc(MLX90614_CRC8POLY);

  // Build the CRC-8 of all bytes to be sent.
  crc.crc8(_addr << 1);
  crc.crc8(cmd);
  crc.crc8(lowByte(data));
  uint8_t _crc8 = crc.crc8(highByte(data));

  // Send the slave address then the command.
  Wire.beginTransmission(_addr);
  Wire.write(cmd);

  // Write the data low byte first.
  Wire.write(lowByte(data));
  Wire.write(highByte(data));

  // Then write the crc.
  Wire.write(_crc8);
  Wire.endTransmission(true);

  // Delay for ensuring write
  delay(MLX90614_WRITE_DELAY);

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

// mlx thermopile settings
#define MLX_OBJECT_EMISSIVITY 0.98

// trigger pins
#define TRIGGER_PIN 2

// laser pins
#define LASER_PIN 8

// operational settings
#define REFRESH_TEMP_READING 200
#define AVERAGING_WINDOW 10

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
  mlx.setEmissivity(MLX_OBJECT_EMISSIVITY);
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