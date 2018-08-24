/********************************************************
 * M5Bala balance car Basic Example
 * Reading encoder and writting the motor via I2C
 ********************************************************/

#include <M5Stack.h>
#include <Wire.h>
#include <Preferences.h>
#include "M5Bala.h"

#include <BlynkSimpleEsp32_BLE.h>
#include <BLEDevice.h>
#include <BLEServer.h>

Preferences preferences;

M5Bala m5bala(Wire);

char auth[] = "23fd9b0986474e97a4976b6d89c9357d";

void auto_tune_gyro_offset();


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

	// Loading the IMU parameters
	if (M5.BtnC.isPressed()) {
		preferences.begin("m5bala-cfg", false);
		auto_tune_gyro_offset();

	} else {
		preferences.begin("m5bala-cfg", true);
		m5bala.imu->setGyroOffsets( preferences.getFloat("gyroXoffset"), 
                                preferences.getFloat("gyroYoffset"), 
                                preferences.getFloat("gyroZoffset"));
	}

	// Blynk start
	Blynk.setDeviceName("M5BOT Blynk");
	Blynk.begin(auth);
}

void loop() {

	// M5Bala balance run
	m5bala.run();

	// Blynk control
	Blynk.run();

	// M5 Loop
	M5.update();
}


BLYNK_WRITE(V0) {
	int16_t joystick_X = param[0].asInt();
	int16_t joystick_Y = param[1].asInt();

	m5bala.move(joystick_Y);
	m5bala.turn(joystick_X);
}


// ================ GYRO offset param ==================
void auto_tune_gyro_offset() {
	M5.Speaker.tone(500, 200);
	delay(300);
	M5.update();
	M5.Lcd.println("Start IMU calculate gyro offsets");
	M5.Lcd.println("DO NOT MOVE A MPU6050...");
	delay(2000);

	m5bala.imu->calcGyroOffsets(true);
	float gyroXoffset = m5bala.imu->getGyroXoffset();
	float gyroYoffset = m5bala.imu->getGyroYoffset();
	float gyroZoffset = m5bala.imu->getGyroZoffset();
	M5.Lcd.println("Done!!!");
	M5.Lcd.print("X : ");M5.Lcd.println(gyroXoffset);
	M5.Lcd.print("Y : ");M5.Lcd.println(gyroYoffset);
	M5.Lcd.print("Z : ");M5.Lcd.println(gyroZoffset);
	M5.Lcd.println("Program will start after 3 seconds");
	M5.Lcd.print("========================================");

	// Save
	preferences.putFloat("gyroXoffset", gyroXoffset);
	preferences.putFloat("gyroYoffset", gyroYoffset);
	preferences.putFloat("gyroZoffset", gyroZoffset);
	preferences.end();
}
