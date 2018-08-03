/********************************************************
 * M5Bala balance car Basic Example
 * Reading encoder and writting the motor via I2C
 ********************************************************/

#include <M5Stack.h>
#include <Wire.h>
#include "M5Bala.h"

M5Bala m5bala(Wire);

void setup() {
  // Power ON Stabilizing...
  delay(500);
  M5.begin();

  // Init I2C
  Wire.begin();
  Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
  delay(500);

  // Display info
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("M5Stack Balance Mode start");

  // Init M5Bala
  m5bala.begin();
  // m5bala.imu->calcGyroOffsets(true);
  m5bala.imu->setGyroOffsets(-2.70, -1.08, 0.87);
}

void loop() {

  // M5Bala run
  m5bala.run();

  // M5 Loop
  M5.update();
}
