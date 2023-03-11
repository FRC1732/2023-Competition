#include <Adafruit_NeoPixel.h>

#define LED_DATA 2

#define NUMPIXELS 69  // number of neopixels in strip
#define NUM_SEG 23    // number of neopixels in a segment
#define DELAY_TIME 200
#define INTENSITY 255

Adafruit_NeoPixel pixels(NUMPIXELS, LED_DATA, NEO_GRB + NEO_KHZ800);

#define DIGITAL_D0 3
#define DIGITAL_D1 4
#define DIGITAL_D2 5
#define DIGITAL_D3 6
#define DIGITAL_D4 7

#define OUTPUT_D0 8
#define OUTPUT_D1 9
#define OUTPUT_D2 10
#define OUTPUT_D3 11
#define OUTPUT_D4 12

int mode = 0;
int timer = 0;

void setup() {
  Serial.begin(250000);

  pinMode(DIGITAL_D0, INPUT_PULLUP);
  pinMode(DIGITAL_D1, INPUT_PULLUP);
  pinMode(DIGITAL_D2, INPUT_PULLUP);
  pinMode(DIGITAL_D3, INPUT_PULLUP);
  pinMode(DIGITAL_D4, INPUT_PULLUP);

  pinMode(OUTPUT_D0, OUTPUT);
  pinMode(OUTPUT_D1, OUTPUT);
  pinMode(OUTPUT_D2, OUTPUT);
  pinMode(OUTPUT_D3, OUTPUT);
  pinMode(OUTPUT_D4, OUTPUT);

  pixels.begin();
}

void loop() {
  bool b0, b1, b2, b3, b4;

  // HIGH is 0, LOW is 1 on the inputs
  b0 = !digitalRead(DIGITAL_D0);
  b1 = !digitalRead(DIGITAL_D1);
  b2 = !digitalRead(DIGITAL_D2);
  b3 = !digitalRead(DIGITAL_D3);
  b4 = !digitalRead(DIGITAL_D4);

  digitalWrite(OUTPUT_D0, b0);
  digitalWrite(OUTPUT_D1, b1);
  digitalWrite(OUTPUT_D2, b2);
  digitalWrite(OUTPUT_D3, b3);
  digitalWrite(OUTPUT_D4, b4);

  mode = ((int)b0 << 0) + ((int)b1 << 1) + ((int)b2 << 2) + ((int)b3 << 3) + ((int)b4 << 4);
  Serial.print("Mode: ");
  Serial.println(mode);

  switch (mode) {
    case 0:
      // no signal sent
      setColor(false, false, false);
      break;

    case 1:
      // Score Low - no game piece
      setColorLow(false, true, false);
      break;

    case 2:
      // Score Mid - no game piece
      setColorMid(false, true, false);
      break;

    case 3:
      // Score High - no game piece
      setColorHigh(false, true, false);
      break;

    case 4:
      // No Score Cube
      setColor(true, false, true);
      break;

    case 5:
      // Cube Low
      setColorLow(true, false, true);
      break;

    case 6:
      // Mid Cube
      setColorMid(true, false, true);
      break;

    case 7:
      // High Cube
      setColorHigh(true, false, true);
      break;

    case 8:
      // No Score Cone
      setColor(true, true, false);
      break;

    case 9:
      // Cone Low
      setColorLow(true, true, false);
      break;

    case 10:
      // Cone Mid
      setColorMid(true, true, false);
      break;

    case 11:
      // Cone High
      setColorHigh(true, true, false);
      break;

    case 12:
      // Blue and Yellow
      setColorBY(false, false, true);
      break;

    case 17:
      // Game piece Captured
      flashFast(false, true, false);
      break;

    default:
      // change nottin'
      break;
  }

  timer++;
  delay(1);
}

void setColor(bool red, bool green, bool blue) {
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
  }
  pixels.show();
}

void blinkColor(bool red, bool green, bool blue) {
  if (timer < 150) {
    setColor(red, green, blue);
  }

  if (timer < 300 && timer > 150) {
    setColor(false, false, false);
  }

  if (timer > 301 || timer < 0) {
    timer = 0;
  }
}

void flashFast(bool red, bool green, bool blue) {
  if (timer < 15) {
    setColor(red, green, blue);
  }

  if (timer < 30 && timer > 15) {
    setColor(false, false, false);
  }

  if (timer > 31 || timer < 0) {
    timer = 0;
  }
}

void setColorLow(bool red, bool green, bool blue) {
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    if (i < NUM_SEG) {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
    }
  }
  pixels.show();
}

void setColorMid(bool red, bool green, bool blue) {
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    //if (i < 46 && i > 22) {
    if (i < NUM_SEG * 2) {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
    }
  }
  pixels.show();
}

void setColorHigh(bool red, bool green, bool blue) {
  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    //if (i < NUMPIXELS && i > 45) {
    pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
    //}
  }
  pixels.show();
}

void setColorBY(bool red, bool green, bool blue) {
  bool invRed = !red;
  bool invGreen = !green;
  bool invBlue = !blue;
  pixels.clear();

  for (int i = 0; i < NUMPIXELS; i++) {
    if ((i / 4) % 2 == 0) {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
    } else {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)invRed, INTENSITY * (int)invGreen * .50, INTENSITY * (int)invBlue));
    }
  }
  pixels.show();
}