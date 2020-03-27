# Dev Guide for Project Setup

## Simulator

We currently use [TinkerCAD](https://www.tinkercad.com/things/2z7LW7in4Lc-fantabulous-esboo/editel?tenant=circuits?sharecode=epiT4VgdNyYcs9Lqs6U2rB50uQc9jo6lYPlmHBFLyK4=).

## Software Flow Diagram

The draw.io diagram is hosted [here](https://app.diagrams.net/?state=%7B%22folderId%22:%221a55Arj23pD1N4WzjrUQBMDq9hy1Yo-RW%22,%22action%22:%22create%22,%22userId%22:%22102256087548180712112%22%7D#G140NqgQfMoGXBQ4lMXQbyQO76sgpE6Sz-).

*Note*: Request for access.

## Hardware Configuration

# Thermal Sensor
**Vendor:** Melexis
**Part No:** MLX90614

Interface with Arduino: I2C aka Two Wire Interface

**Arduino Pins:** A4/A5

# User Trigger
User trigger is a digital input with a pull-up resistor. When the button is pressed the signal goes low.

**Arduino Pin:** D2

**Note: If this pin is multiplexed to Interrupt (INTx), then we can use the trigger to wake-up the device**

# LCD Display
**Vendor:** Generic

**Part Details:** 16x2 Character LCD with Hitachi Controller (HD44780) and Backlight LED

**Interface with Arduino:** 9x Digital Outputs (Bit Banging)

Pin Details are as given below:

| Arduino Pin | LCD Pin/Function   | High                       | Low                         |
|-------------|--------------------|----------------------------|-----------------------------|
| D3          | Backlight Control  | Backlight On               | Backlight Off               |
| D4          | D7                 | 1                          | 0                           |
| D5          | D6                 | 1                          | 0                           |
| D5          | D5                 | 1                          | 0                           |
| D6          | D4                 | 1                          | 0                           |
| D9          | Enable             | LCD Register Access Enable | LCD Register Access Disable |
| D10         | Register Select    | 1                          | 0                           |
| D11         | Brightness Control | Backlight On               | Backlight Off               |


**Note: Pin D3 can be multiplexed PWM OC0. AnalogWrite will work on this pin**

# Serial Interface
Serial Interface is exposed for debugging and calibration

**Pins:** D0/RX and D1/Tx

