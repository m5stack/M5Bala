/********************************************************
 * M5Bala balance car Basic Example
 * Reading encoder and writting the motor via I2C
 ********************************************************/

#include <M5Stack.h>
#include <Wire.h>
#include <NeoPixelBus.h>
#include "M5Bala.h"

M5Bala m5bala(Wire);

// ====================== NeoPixel =======================
const uint16_t PixelCount = 10;
const uint8_t PixelPin = 15; 

#define colorSaturation 10
// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);
RgbColor white(colorSaturation);

void LED_start() {
	strip.Begin();
	for (int i=0; i < 10; i++) {
		strip.SetPixelColor(i, white);
	}
	strip.Show();
}

void draw_waveform() {
	#define MAX_LEN 120
	#define Y_OFFSET 100
	#define X_SCALE  1.8
	static int16_t val_buf[MAX_LEN+3] = {0};
	static uint32_t cnt = 0;

	uint16_t pt = MAX_LEN - (++cnt % MAX_LEN);
	val_buf[pt] = constrain((int16_t)(m5bala.getAngle() * X_SCALE), -50, 50);

	for (int i = 0; i < (MAX_LEN-2); i++) {
		uint16_t now_pt = (pt + i) % (MAX_LEN);
		M5.Lcd.drawLine(i, val_buf[now_pt+1] + Y_OFFSET, i+1, val_buf[now_pt+2] + Y_OFFSET, BLACK);
		M5.Lcd.drawLine(i, val_buf[now_pt]   + Y_OFFSET, i+1, val_buf[now_pt+1] + Y_OFFSET, GREEN);
	}
}

void setup()
{
	// Power ON Stabilizing...
	delay(500);
	M5.begin();

	// Turn on LED BAR
	LED_start();

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
	// m5bala.imu->calcGyroOffsets(true); /、
	// m5bala.imu->setGyroOffsets(-3.09, 0.84, -0.30);
}


void loop()
{
	// LCD display
	static uint32_t print_interval = millis() + 30;
	if (millis() > print_interval) {
		print_interval = millis() + 100;
		M5.Lcd.setCursor(0, 190);
		M5.Lcd.printf("Input  Encoer0: %+4d   Encoer0: %+4d     \r\n", m5bala.getSpeed0(), m5bala.getSpeed1());
		M5.Lcd.printf("Output PWM0: %+4d     PWM1: %+4d       \r\n", m5bala.getOut0(), m5bala.getOut1());
		M5.Lcd.printf("AngleX: %+05.2f\r\n", m5bala.getAngle());
	}

	static uint32_t draw_interval = millis() + 5;
	if (millis() > draw_interval) {
		draw_interval = millis() + 20;
		draw_waveform();
	}

	// M5Bala run
	m5bala.run();

	// M5 Loop
	M5.update();
}
