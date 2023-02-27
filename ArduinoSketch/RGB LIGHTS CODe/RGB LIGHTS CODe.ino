#include <Adafruit_NeoPixel.h>

#define LED_DATA 2

#define NUMPIXELS 69 // number of neopixels in strip
#define DELAY_TIME 200
#define INTENSITY 150

Adafruit_NeoPixel pixels(NUMPIXELS, LED_DATA, NEO_GRB + NEO_KHZ800);

#define DIGITAL_D0 8
#define DIGITAL_D1 9
#define DIGITAL_D2 10
#define DIGITAL_D3 11
#define DIGITAL_D4 12

#define OUTPUT_D0 3
#define OUTPUT_D1 4
#define OUTPUT_D2 5
#define OUTPUT_D3 6
#define OUTPUT_D4 7

int mode = 0;
int timer = 0;

void setup()
{
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

void loop()
{
  bool b0, b1, b2, b3, b4;

  // HIGH is 0, LOW is 1 in the inputs
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

  if (mode == 0)
  {
    setColor(false, false, false);
  }

  // Score Low
  if (mode == 1)
  {
    setColorLow(false, false, true);
  }

  // Score Mid
  if (mode == 2)
  {
    setColorMid(true, false, false);
  }

  // Score High
  if (mode == 3)
  {
    setColorHigh(false, false, true);
  }

  // NoLights Cube
  if (mode == 4)
  {
    setColorTop(true, false, true);
  }

  // Cube Low
  if (mode == 5)
  {
    setColorLow(true, false, true);
  }

  // Mid Cube
  if (mode == 6)
  {
    setColorMid(true, false, true);
  }

  // High Cube
  if (mode == 7)
  {
    setColorHigh(true, false, true);
  }

  // Cone NoLights
  if (mode == 8)
  {  

  }

  // Cone Low
  if (mode == 9)
  {
    setColorLow(true, true, false);
  }

  // Cone Mid
  if (mode == 10)
  {
    setColorMid(true, true, false);
  }

  // Cone High
  if (mode == 11)
  {
    setColorHigh(true, true, false);
  }

  if (mode == 12)
  {
    setColorBY(false, false, true);
  }

  timer++;
  delay(1);
}

void setColor(bool red, bool green, bool blue)
{
  pixels.clear();
  // pixels2.clear();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
    // pixels2.setPixelColor(i, pixels2.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
  }
  pixels.show();
  // pixels2.show();
}

void blinkColor(bool red, bool green, bool blue)
{
  if (timer < 150)
  {
    setColor(red, green, blue);
  }
  if (timer < 300 && timer > 150)
  {
    setColor(false, false, false);
  }
  if (timer > 301)
  {
    timer = 0;
  }
}

void flashFast(bool red, bool green, bool blue)
{
  if (timer < 60)
  {
    setColor(red, green, blue);
  }
  if (timer < 120 && timer > 60)
  {
    setColor(false, false, false);
  }
  if (timer > 121)
  {
    timer = 0;
  }
}
void setColorLow(bool red, bool green, bool blue)
{
  pixels.clear();
  // pixels2.clear();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    if (i < 23)
    {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
    }
  }
  pixels.show();
  // pixels2.show();
}
void setColorMid(bool red, bool green, bool blue)
{
  pixels.clear();
  // pixels2.clear();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    if (i < 46 && i > 22)
    {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
    }
    // pixels2.setPixelColor(i, pixels2.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
  }
  pixels.show();
  // pixels2.show();
}
void setColorHigh(bool red, bool green, bool blue)
{
  pixels.clear();
  // pixels2.clear();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    if (i < NUMPIXELS && i > 45)
    {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
    }
  }
  pixels.show();
  // pixels2.show();
}
void setColorBY(bool red, bool green, bool blue)
{
  bool InvRed = !red;
  bool InvGreen = !green;
  bool InvBlue = !blue;
  pixels.clear();
  // pixels2.clear();
  for (int i = 0; i < NUMPIXELS; i++)
  {
    if ((i / 4) % 2 == 0)
    {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
      // pixels2.setPixelColor(i, pixels2.Color(INTENSITY * (int)red, INTENSITY * (int)green, INTENSITY * (int)blue));
    }
    else
    {
      pixels.setPixelColor(i, pixels.Color(INTENSITY * (int)InvRed, INTENSITY * (int)InvGreen, INTENSITY * (int)InvBlue));
    }
  }
  pixels.show();
  // pixels2.show();
}