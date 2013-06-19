#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9340.h"

#define _rst 5
#define _sclk 13
#define _dc 7
#define _cs 10
#define _mosi 11
#define _miso 12

//Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _mosi, _sclk, _rst, _miso);
Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);

void setup() {
  Serial.begin(9600);
  Serial.println("hi!"); 
 
  tft.begin();

  uint8_t x = tft.readcommand8(ILI9340_RDMODE);

  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9340_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9340_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9340_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9340_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX); 
  
  uint16_t time = millis();
  tft.fillScreen(ILI9340_BLUE);
  time = millis() - time;
  Serial.println(time, DEC);
  
  // large block of text
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ILI9340_WHITE);
  delay(1000);
  // a single pixel
  tft.drawPixel(tft.width()/2, tft.height()/2, ILI9340_GREEN);
  delay(500);

  // line draw test
  testlines(ILI9340_YELLOW);
  delay(500);

  // optimized lines
  testfastlines(ILI9340_RED, ILI9340_BLUE);
  delay(500);

  testdrawrects(ILI9340_GREEN);
  delay(500);

  testfillrects(ILI9340_YELLOW, ILI9340_MAGENTA);
  delay(500);

  tft.fillScreen(ILI9340_BLACK);
  testfillcircles(10, ILI9340_BLUE);
  testdrawcircles(10, ILI9340_WHITE);
  delay(500);

  testroundrects();
  delay(500);

  testtriangles();
  delay(500);

  mediabuttons();
  delay(500);

  Serial.println("done");
  delay(1000);
}

void loop() {
   tft.invertDisplay(true);
   delay(500);
   tft.invertDisplay(false);
   delay(1000);
}

void testlines(uint16_t color) {
  tft.fillScreen(ILI9340_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, 0, tft.width()-1, y, color);
  }

  tft.fillScreen(ILI9340_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, 0, x, tft.height()-1, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, 0, 0, y, color);
  }

  tft.fillScreen(ILI9340_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(0, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(0, tft.height()-1, tft.width()-1, y, color);
  }

  tft.fillScreen(ILI9340_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, x, 0, color);
  }
  for (int16_t y=0; y < tft.height(); y+=6) {
    tft.drawLine(tft.width()-1, tft.height()-1, 0, y, color);
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

void testfastlines(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ILI9340_BLACK);
  for (int16_t y=0; y < tft.height(); y+=5) {
    tft.drawFastHLine(0, y, tft.width(), color1);
  }
  for (int16_t x=0; x < tft.width(); x+=5) {
    tft.drawFastVLine(x, 0, tft.height(), color2);
  }
}

void testdrawrects(uint16_t color) {
  tft.fillScreen(ILI9340_BLACK);
  for (int16_t x=0; x < tft.width(); x+=6) {
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color);
  }
}

void testfillrects(uint16_t color1, uint16_t color2) {
  tft.fillScreen(ILI9340_BLACK);
  for (int16_t x=tft.width()-1; x > 6; x-=6) {
    tft.fillRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color1);
    tft.drawRect(tft.width()/2 -x/2, tft.height()/2 -x/2 , x, x, color2);
  }
}

void testfillcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=radius; x < tft.width(); x+=radius*2) {
    for (int16_t y=radius; y < tft.height(); y+=radius*2) {
      tft.fillCircle(x, y, radius, color);
    }
  }
}

void testdrawcircles(uint8_t radius, uint16_t color) {
  for (int16_t x=0; x < tft.width()+radius; x+=radius*2) {
    for (int16_t y=0; y < tft.height()+radius; y+=radius*2) {
      tft.drawCircle(x, y, radius, color);
    }
  }
}

void testtriangles() {
  tft.fillScreen(ILI9340_BLACK);
  int color = 0xF800;
  int t;
  int w = 63;
  int x = 159;
  int y = 0;
  int z = 127;
  for(t = 0 ; t <= 15; t+=1) {
    tft.drawTriangle(w, y, y, x, z, x, color);
    x-=4;
    y+=4;
    z-=4;
    color+=100;
  }
}

void testroundrects() {
  tft.fillScreen(ILI9340_BLACK);
  int color = 100;
  int i;
  int t;
  for(t = 0 ; t <= 4; t+=1) {
    int x = 0;
    int y = 0;
    int w = 127;
    int h = 159;
    for(i = 0 ; i <= 24; i+=1) {
      tft.drawRoundRect(x, y, w, h, 5, color);
      x+=2;
      y+=3;
      w-=4;
      h-=6;
      color+=1100;
    }
    color+=100;
  }
}

void tftPrintTest() {
  float p = 3.1415926;

  tft.setTextWrap(false);
  tft.fillScreen(ILI9340_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ILI9340_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9340_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ILI9340_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ILI9340_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ILI9340_BLACK);
  tft.setTextColor(ILI9340_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ILI9340_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ILI9340_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ILI9340_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ILI9340_WHITE);
  tft.print(" seconds.");
}

void mediabuttons() {
  // play
  tft.fillScreen(ILI9340_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, ILI9340_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ILI9340_RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, ILI9340_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, ILI9340_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, ILI9340_GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ILI9340_BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, ILI9340_RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, ILI9340_RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ILI9340_GREEN);
}
