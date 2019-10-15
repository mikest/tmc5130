/* TMC5130 UART example

This code demonstrates the usage of the reliable vs. streaming modes
when driving a Trinamic TMC5130 stepper driver using its single-wire interface.

Hardware setup :
A RS485 transceiver must be connected to the Serial1 pins with the TX Enable pin
accessible to the uC

Tie the A output to the TMC5130 I/O voltage with a 1k resistor (see TMC5130 datasheet
Fig 5.2)

The TMC5130 Enable line must be connected to GND to enable the driver.

                                                3.3/5V
                                                  +     +-----------------+
                                            +-----+---+-+ VCC_IO          |
                                            |         | |                 |
                                           +++        | |                 |
+-----------+        +--------------+      | | 1k     +-+ SWSEL           |
|           |        |              |      +++          |                 |
|      RX 0 +--------+ RO           |       |           |                 |
|           |        |            A +-------+-----------+ SWP         NAI ++ open
|           |    +---+ /RE          |                   |                 |
|   TX EN 5 +----+   |            B +-------------------+ SWN             |
|           |    +---+ DE           |                   |                 |
|           |        |          GND +----+         +----+ DRV_ENN         |
|      TX 1 +--------+ DI           |    |         |    |                 |
|           |        |              |    +---------+----+ GND             |
+-----------+        +--------------+                   +-----------------+
 Feather M0               MAX3485                              TMC5130
                       or equivalent


Tested on Adafruit Feather M0 / Teensy 3.x

Copyright (c) 2017 Tom Magnier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Arduino.h>
#include <Estee_TMC5130.h>

const uint8_t UART_TX_EN = 5;   // Differential transceiver TX enable pin

Estee_TMC5130_UART_Transceiver tmc = Estee_TMC5130_UART_Transceiver(UART_TX_EN, Serial1, 1); //Use Serial1 (hardware UART on Feather M0) ; address 1 (NAI input has a pull up resistor)

void setup()
{
  // init serial coms
  Serial.begin(9600);
  while (!Serial); //Wait for serial monitor opening on USB boards

  // Init TMC serial bus @ 500kbps
  Serial1.begin(500000);
  Serial1.setTimeout(1); // TMC5130 should answer back immediately when reading a register.

  // status LED
  pinMode(LED_BUILTIN, OUTPUT);

  //Use reliable mode for initialization to make sure that no commands are skipped
  tmc.setCommunicationMode(Estee_TMC5130_UART::RELIABLE_MODE);

  // This sets the motor currents for HOLD & RUN, as well as the natural motor direction for positive moves
  tmc.begin(10, 20, Estee_TMC5130::NORMAL_MOTOR_DIRECTION);

  // ramp definition
  tmc.setRampMode(Estee_TMC5130_UART::POSITIONING_MODE);
  tmc.setMaxSpeed(200);
  tmc.setAcceleration(500);

  //Switch to streaming mode to send target positions at repeated intervals
  tmc.setCommunicationMode(Estee_TMC5130_UART::STREAMING_MODE);

  Serial.print("Write success rate : ");
  Serial.print(tmc.getWriteSuccessRate() * 100.0);
  Serial.println("%");

  Serial.print("Read success rate : ");
  Serial.print(tmc.getReadSuccessRate() * 100.0);
  Serial.println("%");

  Serial.println("Starting up");
}

void loop()
{
  //Sinusoidal movement
  float period_ms = 5000;
  int amplitude = 100;
  tmc.setTargetPosition(sin(2 * PI * millis() / period_ms) * amplitude);
  delay(5);
}
