#include <Adafruit_NeoPixel.h>

#define LED_DATA 2

#define NUMPIXELS 69  // number of neopixels in strip
#define NUM_SEG 23    // number of neopixels in a segment
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

#define VISION_CYCLE 10
#define VISION_BLOCK 4

#define IDLE_CYCLE 100
#define IDLE_BLOCK 8
uint32_t lowBlue = pixels.Color(0, 0, INTENSITY / 3);
uint32_t highBlue = pixels.Color(0, 0, INTENSITY);
uint32_t lowGold = pixels.Color(INTENSITY / 3, INTENSITY / 6, 0);
uint32_t highGold = pixels.Color(INTENSITY, INTENSITY / 2, 0);

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
      idleMode();
      break;

    case 1:
    case 1 + 16:
      // Score Low - no game piece
      setColorLow(false, true, false, b4);
      break;

    case 2:
    case 2 + 16:
      // Score Mid - no game piece
      setColorMid(false, true, false, b4);
      break;

    case 3:
    case 3 + 16:
      // Score High - no game piece
      setColorHigh(false, true, false, b4);
      break;

    case 4:
    case 4 + 16:
      // No Score Cube
      setColor(true, false, true);
      break;

    case 5:
    case 5 + 16:
      // Cube Low
      setColorLow(true, false, true, b4);
      break;

    case 6:
    case 6 + 16:
      // Mid Cube
      setColorMid(true, false, true, b4);
      break;

    case 7:
    case 7 + 16:
      // High Cube
      setColorHigh(true, false, true, b4);
      break;

    case 8:
    case 8 + 16:
      // No Score Cone
      setColor(true, true, false);
      break;

    case 9:
    case 9 + 16:
      // Cone Low
      setColorLow(true, true, false, b4);
      break;

    case 10:
    case 10 + 16:
      // Cone Mid
      setColorMid(true, true, false, b4);
      break;

    case 11:
    case 11 + 16:
      // Cone High
      setColorHigh(true, true, false, b4);
      break;

    case 28:
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

void setColorLow(bool red, bool green, bool blue, bool withVis) {
  setColorPosition(red, green, blue, NUM_SEG, withVis);
}

void setColorMid(bool red, bool green, bool blue, bool withVis) {
  setColorPosition(red, green, blue, NUM_SEG * 2, withVis);
}

void setColorHigh(bool red, bool green, bool blue, bool withVis) {
  setColorPosition(red, green, blue, NUMPIXELS, withVis);
}

void setColorPosition(bool red, bool green, bool blue, int pos, bool withVis) {
  if (timer > VISION_BLOCK * VISION_CYCLE * NUMPIXELS || timer < 0) {
    timer = 0;
  }

  int offset = timer / VISION_CYCLE;
  int visPos = offset % (NUMPIXELS - VISION_BLOCK);

  pixels.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    if (i < pos) {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
    }

    if (withVis) {
      if (i >= visPos && i < visPos + VISION_BLOCK) {
        pixels.setPixelColor(i, pixels.Color(INTENSITY, INTENSITY, INTENSITY));
      }
    }
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

void idleMode() {
  pixels.clear();

  if (timer > IDLE_BLOCK * IDLE_CYCLE || timer < 0) {
    timer = 0;
  }

  int offset = timer / IDLE_CYCLE;

  for (int i = 0; i < NUMPIXELS; i++) {
    uint32_t color;
    int pos = (offset + i) % IDLE_BLOCK;
    if (pos == 0 || pos == 3) {
      color = lowBlue;
    } else if (pos == 1 || pos == 2) {
      color = highBlue;
    } else if (pos == 4 || pos == 7) {
      color = lowGold;
    } else if (pos == 5 || pos == 6) {
      color = highGold;
    } else {
      color = pixels.Color(0, 0, 0);
    }
    pixels.setPixelColor(i, color);
  }

  pixels.show();
}