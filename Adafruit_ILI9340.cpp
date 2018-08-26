/***************************************************
  This is an Arduino Library for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480
 
  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ILI9340.h"
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

#if defined(__SAM3X8E__)
#include <include/pio.h>
  #define SET_BIT(port, bitMask) (port)->PIO_SODR |= (bitMask)
  #define CLEAR_BIT(port, bitMask) (port)->PIO_CODR |= (bitMask)
  #define USE_SPI_LIBRARY
#endif
#ifdef __AVR__
  #define SET_BIT(port, bitMask) *(port) |= (bitMask)
  #define CLEAR_BIT(port, bitMask) *(port) &= ~(bitMask)
#endif
#if (defined(__arm__) && defined(CORE_TEENSY)) || defined(ESP8266)
  #define USE_SPI_LIBRARY
  #define SET_BIT(port, bitMask) digitalWrite(*(port), HIGH);
  #define CLEAR_BIT(port, bitMask) digitalWrite(*(port), LOW);
#endif

// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t mosi,
				   uint8_t sclk, uint8_t rst, uint8_t miso) : Adafruit_GFX(ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
  m_tiny = true;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t rst) : Adafruit_GFX(ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
  m_tiny = true;
}

void Adafruit_ILI9340::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
#ifdef __AVR__
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
#endif
#if defined(USE_SPI_LIBRARY)
    SPI.transfer(c);
#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
        //digitalWrite(_mosi, HIGH); 
        SET_BIT(mosiport, mosipinmask);
      } else {
        //digitalWrite(_mosi, LOW); 
        CLEAR_BIT(mosiport, mosipinmask);
      }
      //digitalWrite(_sclk, HIGH);
      SET_BIT(clkport, clkpinmask);
      //digitalWrite(_sclk, LOW);
      CLEAR_BIT(clkport, clkpinmask);
    }
  }
}


void Adafruit_ILI9340::writecommand(uint8_t c) {
  CLEAR_BIT(dcport, dcpinmask);
  //digitalWrite(_dc, LOW);
  CLEAR_BIT(clkport, clkpinmask);
  //digitalWrite(_sclk, LOW);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);

  spiwrite(c);

  SET_BIT(csport, cspinmask);
  //digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340::writedata(uint8_t c) {
  SET_BIT(dcport,  dcpinmask);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(clkport, clkpinmask);
  //digitalWrite(_sclk, LOW);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);
  
  spiwrite(c);

  //digitalWrite(_cs, HIGH);
  SET_BIT(csport, cspinmask);
} 

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Adafruit_ILI9340::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void Adafruit_ILI9340::begin(void) {
  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
#ifdef __AVR__
  csport    = portOutputRegister(digitalPinToPort(_cs));
  dcport    = portOutputRegister(digitalPinToPort(_dc));
#endif
#if defined(__SAM3X8E__)
  csport    = digitalPinToPort(_cs);
  dcport    = digitalPinToPort(_dc);
#endif
#if (defined(__arm__) && defined(CORE_TEENSY)) || defined(ESP8266)
  mosiport = &_mosi;
  clkport = &_sclk;
  rsport = &_rst;
  csport    = &_cs;
  dcport    = &_dc;
#endif
  cspinmask = digitalPinToBitMask(_cs);
  dcpinmask = digitalPinToBitMask(_dc);

  if(hwSPI) { // Using hardware SPI
    SPI.begin();
#if defined(__AVR__) || defined(ESP8266)
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
#endif
#if defined(__SAM3X8E__)
    SPI.setClockDivider(11); // 85MHz / 11 = 7.6 MHz (full! speed!)
#endif
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
#ifdef __AVR__
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
#endif
#if defined(__SAM3X8E__)
    clkport     = digitalPinToPort(_sclk);
    mosiport    = digitalPinToPort(_mosi);
#endif
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosipinmask = digitalPinToBitMask(_mosi);
    CLEAR_BIT(clkport, clkpinmask);
    CLEAR_BIT(mosiport, mosipinmask);
  }

  // toggle RST low to reset

  digitalWrite(_rst, HIGH);
  delay(5);
  digitalWrite(_rst, LOW);
  delay(20);
  digitalWrite(_rst, HIGH);
  delay(150);

  /*
  uint8_t x = readcommand8(ILI9340_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9340_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
  */

  //if(cmdList) commandList(cmdList);
  
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);  
  writedata(0x00); 
  writedata(0XC1); 
  writedata(0X30); 

  writecommand(0xED);  
  writedata(0x64); 
  writedata(0x03); 
  writedata(0X12); 
  writedata(0X81); 
 
  writecommand(0xE8);  
  writedata(0x85); 
  writedata(0x00); 
  writedata(0x78); 

  writecommand(0xCB);  
  writedata(0x39); 
  writedata(0x2C); 
  writedata(0x00); 
  writedata(0x34); 
  writedata(0x02); 
 
  writecommand(0xF7);  
  writedata(0x20); 

  writecommand(0xEA);  
  writedata(0x00); 
  writedata(0x00); 
 
  writecommand(ILI9340_PWCTR1);    //Power control 
  writedata(0x23);   //VRH[5:0] 
 
  writecommand(ILI9340_PWCTR2);    //Power control 
  writedata(0x10);   //SAP[2:0];BT[3:0] 
 
  writecommand(ILI9340_VMCTR1);    //VCM control 
  writedata(0x3e); //�Աȶȵ���
  writedata(0x28); 
  
  writecommand(ILI9340_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9340_MADCTL);    // Memory Access Control 
  writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);

  writecommand(ILI9340_PIXFMT);    
  writedata(0x55); 
  
  writecommand(ILI9340_FRMCTR1);    
  writedata(0x00);  
  writedata(0x18); 
 
  writecommand(ILI9340_DFUNCTR);    // Display Function Control 
  writedata(0x08); 
  writedata(0x82);
  writedata(0x27);  
 
  writecommand(0xF2);    // 3Gamma Function Disable 
  writedata(0x00); 
 
  writecommand(ILI9340_GAMMASET);    //Gamma curve selected 
  writedata(0x01); 
 
  writecommand(ILI9340_GMCTRP1);    //Set Gamma 
  writedata(0x0F); 
  writedata(0x31); 
  writedata(0x2B); 
  writedata(0x0C); 
  writedata(0x0E); 
  writedata(0x08); 
  writedata(0x4E); 
  writedata(0xF1); 
  writedata(0x37); 
  writedata(0x07); 
  writedata(0x10); 
  writedata(0x03); 
  writedata(0x0E); 
  writedata(0x09); 
  writedata(0x00); 
  
  writecommand(ILI9340_GMCTRN1);    //Set Gamma 
  writedata(0x00); 
  writedata(0x0E); 
  writedata(0x14); 
  writedata(0x03); 
  writedata(0x11); 
  writedata(0x07); 
  writedata(0x31); 
  writedata(0xC1); 
  writedata(0x48); 
  writedata(0x08); 
  writedata(0x0F); 
  writedata(0x0C); 
  writedata(0x31); 
  writedata(0x36); 
  writedata(0x0F); 

  writecommand(ILI9340_SLPOUT);    //Exit Sleep 
  delay(120); 		
  writecommand(ILI9340_DISPON);    //Display on 
}


void Adafruit_ILI9340::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
 uint16_t y1) {

  writecommand(ILI9340_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART 
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9340_PASET); // Row addr set
  writedata(y0>>8);
  writedata(y0);     // YSTART
  writedata(y1>>8);
  writedata(y1);     // YEND

  writecommand(ILI9340_RAMWR); // write to RAM
}


void Adafruit_ILI9340::pushColor(uint16_t color) {
  //digitalWrite(_dc, HIGH);
  SET_BIT(dcport, dcpinmask);
  //digitalWrite(_cs, LOW);
  CLEAR_BIT(csport, cspinmask);

  spiwrite(color >> 8);
  spiwrite(color);

  SET_BIT(csport, cspinmask);
  //digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  //digitalWrite(_dc, HIGH);
  SET_BIT(dcport, dcpinmask);
  //digitalWrite(_cs, LOW);
  CLEAR_BIT(csport, cspinmask);

  spiwrite(color >> 8);
  spiwrite(color);

  SET_BIT(csport, cspinmask);
  //digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;

  if((y+h-1) >= _height) 
    h = _height-y;

  setAddrWindow(x, y, x, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SET_BIT(dcport, dcpinmask);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  SET_BIT(csport, cspinmask);
  //digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  SET_BIT(dcport, dcpinmask);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_dc, HIGH);
  //digitalWrite(_cs, LOW);
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  SET_BIT(csport, cspinmask);
  //digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::fillScreen(uint16_t color) {
  fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9340::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);

  uint8_t hi = color >> 8, lo = color;

  SET_BIT(dcport, dcpinmask);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
  SET_BIT(csport, cspinmask);
}

void Adafruit_ILI9340::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t wReal, int16_t h,
			      uint16_t color, uint16_t bgcolor) {

  // fillRect copy, except sending color/bgcolor B/W bitmapdata
  // bitmap read as bit stream - no byte align @ row level !
  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  int16_t h2 = h; // start with requested size
  if((x + w - 1) >= _width)  wReal = _width  - x;
  if((y + h - 1) >= _height) h2 = _height - y;

  setAddrWindow(x, y, x+wReal-1, y+h2-1); // smaller window

  uint8_t hi = color >> 8, lo = color;
  uint8_t hiBg = bgcolor >> 8, loBg = bgcolor; // prepare also background color

  SET_BIT(dcport, dcpinmask);
  //digitalWrite(_dc, HIGH);
  CLEAR_BIT(csport, cspinmask);
  //digitalWrite(_cs, LOW);

  int16_t i, j;

  unsigned char mask = 128;
  char oneB = *(bitmap++);
  for(j=0; j<h; j++) {
	for(i=0; i<w; i++ ) {
		if((j<h2) && (i<=wReal)) { // draw only when inside bitmap && screen
			if(oneB & mask) { // foreground
				spiwrite(hi);
				spiwrite(lo);
			} else { // background
				spiwrite(hiBg);
				spiwrite(loBg);
			}
		}
		mask >>= 1;
		if(!mask) { // bits sent - take next byte
			mask = 128;
			oneB = *(bitmap++);
		}
	}
  }
  //digitalWrite(_cs, HIGH);
  SET_BIT(csport, cspinmask);
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9340::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


void Adafruit_ILI9340::setRotation(uint8_t m) {

  writecommand(ILI9340_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
     break;
   case 1:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  case 2:
    writedata(ILI9340_MADCTL_MY | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
    break;
   case 3:
     writedata(ILI9340_MADCTL_MV | ILI9340_MADCTL_MY | ILI9340_MADCTL_MX | ILI9340_MADCTL_BGR);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  }
}


void Adafruit_ILI9340::invertDisplay(boolean i) {
  writecommand(i ? ILI9340_INVON : ILI9340_INVOFF);
}


////////// stuff not actively being used, but kept for posterity


uint8_t Adafruit_ILI9340::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#ifdef __AVR__
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
#endif
#if defined(USE_SPI_LIBRARY)
    r = SPI.transfer(0x00);
#endif
  } else {

    for (uint8_t i=0; i<8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
	r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);
  
  return r;
}

 uint8_t Adafruit_ILI9340::readdata(void) {
   digitalWrite(_dc, HIGH);
   digitalWrite(_cs, LOW);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   
   return r;
}
 

 uint8_t Adafruit_ILI9340::readcommand8(uint8_t c) {
   digitalWrite(_dc, LOW);
   digitalWrite(_sclk, LOW);
   digitalWrite(_cs, LOW);
   spiwrite(c);
 
   digitalWrite(_dc, HIGH);
   uint8_t r = spiread();
   digitalWrite(_cs, HIGH);
   return r;
}


 
/*

 uint16_t Adafruit_ILI9340::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t Adafruit_ILI9340::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */

#if ARDUINO < 100
size_t Adafruit_ILI9340::write(uint8_t c) {
#else
void Adafruit_ILI9340::write(uint8_t c) {
#endif
	int charWidth;
  if (c == '\n') {
    cursor_y += textsize*9-2;
    cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
	charWidth = cursor_x;
    drawChar(cursor_x + corner_x, cursor_y + corner_y, c, textcolor, textbgcolor, textsize);
	if(!m_proportional) {
		cursor_x += textsize*6;
		if(!textsize) cursor_x += 6;
	} else charWidth = cursor_x - charWidth;
    if (wrap && (cursor_x > (_width - textsize*6))) {
      cursor_y += textsize*9-2;
      cursor_x = 0;
    }
  }
	if(!m_proportional)
		return 1;
	else return charWidth;
}

// draw a character
#include "glcdfont.c"
void Adafruit_ILI9340::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t color, uint16_t bg, uint8_t size) {

  if((x >= _width)            || // Clip right
     (y >= _height)           || // Clip bottom
     ((x + 8 * size - 1) < 0) || // Clip left
     ((y + 12 * size - 1) < 0))  // Clip top
    return;

	c-='!';
	const unsigned char* begin = m_font;
	unsigned char chars = pgm_read_byte(begin+2), width = pgm_read_byte(begin+3), height = pgm_read_byte(begin+4);
	if(c==255) {
		fillRect(x, y, width*8, height, bg);
		if(m_proportional) cursor_x += width*8*5/8;
		return;
	}
	const unsigned char* rowTabIdx = begin + 5;
	const unsigned char* rowCollTabIdx = begin + 5 + ((chars*height+7)>>3); // 423 pro 94*36/8
	const unsigned char* charDataTabIdx = rowCollTabIdx + pgm_read_byte(begin+1) + (pgm_read_byte(begin)<<8);
	int skipRows = c*height;
	unsigned char byteRows, byteColls = pgm_read_byte(rowCollTabIdx);
	rowCollTabIdx++;
	unsigned char rowTabMask;
	unsigned char rowCollTabMask = 128;
	rowTabMask = 128;
	byteRows = pgm_read_byte(rowTabIdx++);
	while(skipRows--) {
		if(byteRows & rowTabMask) {
			for(char i=0;i<width;i++) {
				if(byteColls & rowCollTabMask) charDataTabIdx++;
				rowCollTabMask >>= 1;
				if(!rowCollTabMask) {
					rowCollTabMask = 128;
					byteColls = pgm_read_byte(rowCollTabIdx);
					rowCollTabIdx++;
				}
			}
		}
		rowTabMask >>= 1;
		if(!rowTabMask) {
			rowTabMask = 128;
			byteRows = pgm_read_byte(rowTabIdx);
			rowTabIdx++;
		}
	}
	if(size > 1) size /= 4;
	unsigned char *dataBuf = new uint8_t[height*width];
	memset(dataBuf,0,height*width);
	char line = 0;
	for(char j=0;j<height;j++) { // read data
		if(byteRows & rowTabMask) {
			for(char i=0;i<width;i++) {
				if(byteColls & rowCollTabMask) { dataBuf[j*width+i] = pgm_read_byte(charDataTabIdx); charDataTabIdx++; }
				rowCollTabMask >>= 1;
				if(!rowCollTabMask) {
					rowCollTabMask = 128;
					byteColls = pgm_read_byte(rowCollTabIdx);
					rowCollTabIdx++;
				}
			}
		}
		rowTabMask >>= 1;
		if(!rowTabMask) {
			rowTabMask = 128;
			byteRows = pgm_read_byte(rowTabIdx);
			rowTabIdx++;
		}
	}
	unsigned int b;
	for(char j=0;j<height;j++) { // unxor data
		b=0; // optional row XOR >> 1 decoding
		for(char i=0;i<width;i++) {
			if(b&1) b = 128; else b = 0;
			b ^= dataBuf[j*width+i];
			// 7 76 65 54 43 32 21 10 ... 07
			//    7 76 65 54 43 32 21 ... 07
			// 7  6 75 64 53 42 31 20
			b ^= (b >> 1);
			// 7  6 75 64 53 42 31 20
			//      7  6  75 64 53 42
			// 7  6  5  4 73 62 51 40
			b ^= (b >> 2);
			// 7  6  5  4 73 62 51 40
			//            7  6  5  4
			// 7  6  5  4  3  2  1  0 ...  0^07=7
			b ^= (b >> 4);
			dataBuf[j*width+i] = b;
		} // optional row XOR >> 1 decoding
		if(j) for(char i=0;i<width;i++) {
				dataBuf[j*width+i] ^= dataBuf[(j-1)*width+i];
		}
	} // unxor data
	char skipLeft = 0, realWidth = width*8-1;
	if(m_proportional) {
		unsigned char *rowOr = new uint8_t[width];
		memset(rowOr, 0, width);
		for(char j=0;j<height;j++) { // or data
			for(char i=0;i<width;i++) {
				rowOr[i] |= dataBuf[j*width+i];
			}
		}
		while((skipLeft < realWidth) && !(rowOr[(skipLeft>>3)] & (128>>(skipLeft&7))))  skipLeft++;
		uint16_t overFlow = 0;
		if(skipLeft) {
			if(skipLeft > 7) skipLeft = 8;
			for(char j=0;j<height;j++) {
				for(char i=width-1;i > -1;i--) {
					overFlow >>= 8;
					overFlow = (dataBuf[j*width+i] << skipLeft) | overFlow;
					dataBuf[j*width+i] = (overFlow & 255);
				}
				overFlow = 0;
			}
		}
		while(!(rowOr[(realWidth>>3)] & (128>>(realWidth&7))))  realWidth--;
		delete[] rowOr;
		cursor_x += realWidth - skipLeft + 3;

		unsigned char *rowPtr = dataBuf;
	}
	drawBitmap(x,y,dataBuf,width<<3,realWidth,height,color,bg);
	delete[] dataBuf;
}
