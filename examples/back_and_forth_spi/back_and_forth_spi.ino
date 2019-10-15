#include <Arduino.h>
#include <SPI.h>
#include <Estee_TMC5130.h>

uint8_t TMC_CS = PIN_A5;  // Chip select, Active LOW. next to SCK/MOSI/MISO on the Adafruit Feather M0
uint8_t TMC_EN = 10;      // Drive enable pin. Active LOW. Optional, you can also just tie to ground
uint8_t BLE_CS = 8;       // Hidden pin #8 which controls the SS line for the Bluetooth module SPI comms

Estee_TMC5130_SPI tmc = Estee_TMC5130_SPI(TMC_CS);

void setup()
{
  // init serial coms
  Serial.begin(9600);

  SPI.begin();

  // chip select & drive enable
  pinMode(TMC_EN, OUTPUT);
  digitalWrite(TMC_EN, HIGH); // disabled for start
  pinMode(TMC_CS, OUTPUT);
  digitalWrite(TMC_EN, HIGH);

  // disable BLE talking on SPI bus
#if defined(ARDUINO_SAMD_ZERO)
  pinMode(BLE_CS, OUTPUT);
  digitalWrite(BLE_CS, HIGH);
#endif

  // This sets the motor currents for HOLD & RUN, as well as the natural motor direction for positive moves
  tmc.begin(4, 4, Estee_TMC5130::NORMAL_MOTOR_DIRECTION);

  // drive *MUST* be disabled when testing frequency scaling
//  digitalWrite(TMC_EN, HIGH);
//  float freq = tmc.updateFrequencyScaling();
//  Serial.print("frequency scaling: ");
//  Serial.println(freq);
//  delay(5000);

  // ramp definition
  tmc.setRampMode(Estee_TMC5130::POSITIONING_MODE);
  tmc.setMaxSpeed(200);
  tmc.setRampSpeeds(0, 0.1, 100); //Start, stop, threshold speeds
  tmc.setAccelerations(250, 350, 500, 700); //AMAX, DMAX, A1, D1

  Serial.println("starting up");

  // enable the drive
  digitalWrite(TMC_EN, LOW);
}

// periodic delay
uint32_t t_echo = millis();
uint32_t t_dirchange = millis();
bool dir = false;

void loop()
{
  uint32_t now = millis();

  // every n seconds or so...
  if ( now - t_dirchange > 3000 )
  {
    t_dirchange = now;

    // reverse direction
    dir = !dir;
    tmc.setTargetPosition(dir ? 200 : 0);  // 1 full rotation = 200s/rev
  }

  // print out current position
  if( now - t_echo > 100 )
  {
    t_echo = now;

    // get the current target position
    int32_t xactual = tmc.getCurrentPosition();
    float vactual = tmc.getCurrentSpeed();

    Serial.print("xpos,v:");
    Serial.print(xactual);
    Serial.print(", ");
    Serial.println(vactual);
  }

}
