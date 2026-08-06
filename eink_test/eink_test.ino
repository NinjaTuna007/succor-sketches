#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ThinkInk.h>

static constexpr int EPD_DC    = 10;
static constexpr int EPD_CS    = 4;
static constexpr int SRAM_CS   = 3;
static constexpr int EPD_RESET = 5;
static constexpr int EPD_BUSY  = -1;

ThinkInk_213_Tricolor_MFGNR display(
  EPD_DC,
  EPD_RESET,
  EPD_CS,
  SRAM_CS,
  EPD_BUSY,
  &SPI
);

void setup()
{
  Serial.begin(115200);

  const uint32_t start = millis();
  while (!Serial && millis() - start < 2000) {
    delay(10);
  }

  Serial.println("BOOT");

  // Deselect both SPI devices before starting SPI.
  pinMode(EPD_CS, OUTPUT);
  digitalWrite(EPD_CS, HIGH);

  pinMode(SRAM_CS, OUTPUT);
  digitalWrite(SRAM_CS, HIGH);

  // Explicit hardware reset.
  pinMode(EPD_RESET, OUTPUT);
  digitalWrite(EPD_RESET, HIGH);
  delay(100);
  digitalWrite(EPD_RESET, LOW);
  delay(20);
  digitalWrite(EPD_RESET, HIGH);
  delay(500);

  SPI.begin();

  Serial.println("Calling display.begin()");
  display.begin(THINKINK_TRICOLOR);

  display.setRotation(1);
  display.setTextWrap(false);

  display.clearBuffer();
  display.fillScreen(EPD_WHITE);

  const int16_t w = display.width();
  const int16_t h = display.height();

  display.fillRect(0, 0, w / 2, h, EPD_BLACK);
  display.fillRect(w / 2, 0, w - w / 2, h, EPD_RED);

  display.setTextSize(2);

  display.setTextColor(EPD_WHITE);
  display.setCursor(15, 45);
  display.print("BLACK");

  display.setTextColor(EPD_BLACK);
  display.setCursor(w / 2 + 15, 45);
  display.print("RED");

  Serial.println("BEFORE display()");
  const uint32_t refresh_start = millis();

  display.display();

  Serial.print("AFTER display(), elapsed ms = ");
  Serial.println(millis() - refresh_start);
}

void loop()
{
}