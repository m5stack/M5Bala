/********************************************************
 * M5Bala balance car Basic Example
 * Reading encoder and writting the motor via I2C
 ********************************************************/

#include <M5Stack.h>
#include <NeoPixelBus.h>
#include <Wire.h>
#include <Preferences.h>
#include "M5Bala.h"

Preferences preferences;

M5Bala m5bala(Wire);

// ==================== NeoPixel =====================
const uint16_t PixelCount = 10;
const uint8_t PixelPin = 15;

#define colorSaturation 10
// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
RgbColor white(colorSaturation);

void LED_start() {
	strip.Begin();
	for (int i = 0; i < 10; i++) {
		strip.SetPixelColor(i, white);
	}
	strip.Show();
}

// ================ Draw Angle Wavefrom =================
void draw_waveform() {
	#define MAX_LEN 120
	#define X_OFFSET 0
	#define Y_OFFSET 100
	#define X_SCALE 3
	static int16_t val_buf[MAX_LEN] = {0};
	static int16_t pt = MAX_LEN - 1;

	val_buf[pt] = constrain((int16_t)(m5bala.getAngle() * X_SCALE), -80, 80);
	if (--pt < 0) {
		pt = MAX_LEN - 1;
	}

	for (int i = 1; i < (MAX_LEN); i++) {
		uint16_t now_pt = (pt + i) % (MAX_LEN);
		M5.Lcd.drawLine(i, val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, i + 1,
							val_buf[(now_pt + 2) % MAX_LEN] + Y_OFFSET, TFT_BLACK);
		if (i < MAX_LEN - 1)
			M5.Lcd.drawLine(i, val_buf[now_pt] + Y_OFFSET, i + 1,
							val_buf[(now_pt + 1) % MAX_LEN] + Y_OFFSET, TFT_GREEN);
	}
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


void setup() {
	// Power ON Stabilizing...
	delay(500);
	M5.begin();
	M5.setPowerBoostKeepOn(false);

	// Turn on LED BAR
	LED_start();

	// Init I2C
	Wire.begin();
	Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
	delay(500);

	// Display info
	M5.Lcd.setTextFont(2);
	M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
	M5.Lcd.println("M5Stack Balance Mode start");

	// Init M5Bala
	m5bala.begin();

	// Loading the IMU parameters
	if (M5.BtnC.isPressed()) {
		preferences.begin("m5bala-cfg", false);
		preferences.putBool("have_cfg", true);
		auto_tune_gyro_offset();

	} else {
		preferences.begin("m5bala-cfg", true);
		if (preferences.getBool("have_cfg")) {
			m5bala.imu->setGyroOffsets( preferences.getFloat("gyroXoffset"), 
										preferences.getFloat("gyroYoffset"), 
										preferences.getFloat("gyroZoffset"));
		}
	}
	preferences.end();
}

void loop() {

	// LCD display
	static uint32_t print_interval = millis() + 30;
	if (millis() > print_interval) {
		print_interval = millis() + 100;
		M5.Lcd.setCursor(0, 190);
		M5.Lcd.printf("Input  Encoer0: %+4d  Encoer1: %+4d    \r\n", 
								m5bala.getSpeed0(), m5bala.getSpeed1());
		M5.Lcd.printf("Output PWM0: %+4d     PWM1: %+4d    \r\n", 
								m5bala.getOut0(), m5bala.getOut1());
		M5.Lcd.printf("AngleX: %+05.2f\r\n", m5bala.getAngle());
	}

	// Draw the waveform
	static uint32_t draw_interval = millis() + 5;
	if (millis() > draw_interval) {
		draw_interval = millis() + 20;
		draw_waveform();
	}

	// M5Bala balance run
	m5bala.run();

	// M5 Loop
	M5.update();
}
