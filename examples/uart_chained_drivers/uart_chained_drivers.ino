/* Multiple TMC5130 in UART mode example

This code demonstrates the usage of a chain of Trinamic TMC5130 stepper drivers

Hardware setup :
A RS485 transceiver must be connected to the Serial1 pins with the TX Enable pin
accessible to the uC

Tie the A output to the TMC5130 I/O voltage with a 1k resistor (see TMC5130 datasheet
Fig 5.2). A single resistor is needed for a chain of TMC5130.

The TMC5130 Enable line must be connected to GND to enable the driver.

Following TMC5130s must be connected in the same way, sharing SWP, SWN and GND.
A strategy is necessary for differentiating the slaves at startup :
 - either connect the first driver NAI input to GND and connect the next NAI to NAO
 - or use a bus switch on SWP / SWN after each of the TMC5130s, triggered by NAO. For example TI TS3A4742 is a suitable chip.

See TMC5130 datasheet §5.4 for details.
This code uses the bus switch strategy with NAI left open.

                                      VCC_IO
                                      3.3/5V
                                         +
                                         |
                                        +++
+-----------+        +--------------+   | | 1k     +--------------------+
|           |        |              |   +++        |                    |
|      RX 0 +--------+ RO           |    |         |                    |
|           |        |            A +----+---------+ SWP_IN     SWP_OUT +----+ ++
|           |    +---+ /RE          |              |                    |
|   TX EN 5 +----+   |            B +--------------+ SWN_IN     SWN_OUT +----+ ++
|           |    +---+ DE           |              |                    |
|           |        |          GND +--------------+ GND            GND +----+ ++
|      TX 1 +--------+ DI           |              |                    |
|           |        |              |              |                    |
+-----------+        +--------------+              +--------------------+
 Feather M0               MAX3485                   TMC5130 & bus switch
                       or equivalent


Tested on Adafruit Feather M0.

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
const int MAX_MOTOR_COUNT = 4;

Estee_TMC5130_UART_Transceiver motors[MAX_MOTOR_COUNT];
int motorCount = 0;

void setup()
{
  // init serial coms
  Serial.begin(9600);
  while(!Serial);

  // Init TMC serial bus @ 500kbps
  Serial1.begin(500000);
  Serial1.setTimeout(1); // TMC5130 should answer back immediately when reading a register.

  // status LED
  pinMode(LED_BUILTIN, OUTPUT);

  // TMC5130 research on bus
  Serial.println("Starting to look for TMC5130.\n");
  bool chainEnd = false;
  while (!chainEnd && motorCount < MAX_MOTOR_COUNT)
  {
    //Use Serial1 (hardware UART on Feather M0) ; address 1 (NAI input has a pull up resistor)
    motors[motorCount] = Estee_TMC5130_UART_Transceiver(UART_TX_EN, Serial1, 1);

    //Try to query a TMC5130 at address 1 (default)
    Estee_TMC5130_UART::ReadStatus readStatus;
    uint32_t gconf = motors[motorCount].readRegister(TMC5130_Reg::GCONF, &readStatus);

    switch (readStatus)
    {
      case Estee_TMC5130_UART::SUCCESS:
      {
        Serial.println("Found TMC5130 @ address 1.");

        //Addressing
        uint8_t address = 254-motorCount;
        motors[motorCount].setSlaveAddress(address, true);
        Serial.print("New address : ");
        Serial.println(address);
        Serial.println();

        //Motor init
        motors[motorCount].begin(10, 20, Estee_TMC5130::NORMAL_MOTOR_DIRECTION);
        motors[motorCount].setRampMode(Estee_TMC5130::POSITIONING_MODE);
        motors[motorCount].setMaxSpeed(200);
        motors[motorCount].setAcceleration(500);

        //Turn on the bus switch to get access to the next board.
        motors[motorCount].writeRegister(TMC5130_Reg::IO_INPUT_OUTPUT, 0); //set NAO low
        motors[motorCount].resetCommunication();
        motorCount++;
      }
      break;

      case Estee_TMC5130_UART::NO_REPLY:
      Serial.println("No more TMC5130 found.");
      chainEnd = true;
      break;

      case Estee_TMC5130_UART::BAD_CRC:
      Serial.println("A TMC5130 replied with a bad CRC. Trying again."); //TODO keep a count of failed attempts.
      motors[motorCount].resetCommunication();
      break;
    }
  }

  Serial.println("Starting up");
}

// periodic delay
uint32_t t_echo = millis();
uint32_t t_dirchange = millis();
bool dir = false;

void loop()
{
  uint32_t now = millis();

  // every n seconds or so...
  if ( now - t_dirchange > 8000 )
  {
    t_dirchange = now;

    // reverse direction
    dir = !dir;
    for (int i = 0; i < motorCount; i++)
    {
      motors[i].setTargetPosition(dir ? 200 * (i+1) : 0);  // 1 full rotation = 200s/rev
    }

    digitalWrite(LED_BUILTIN, dir);
  }
}
