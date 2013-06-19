/*************************************************** 

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
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

// Constructor when using software SPI.  All output pins are configurable.
Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t mosi,
				   uint8_t sclk, uint8_t rst, uint8_t miso) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9340::Adafruit_ILI9340(uint8_t cs, uint8_t dc, uint8_t rst) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
}


void Adafruit_ILI9340::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI) {
    SPDR = c;
    while(!(SPSR & _BV(SPIF)));
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) {
	//digitalWrite(_mosi, HIGH); 
	*mosiport |=  mosipinmask;
      } else {
	//digitalWrite(_mosi, LOW); 
	*mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
  }
}


void Adafruit_ILI9340::writecommand(uint8_t c) {
  *dcport &=  ~dcpinmask;
  //digitalWrite(_dc, LOW);
  *clkport &= ~clkpinmask;
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  spiwrite(c);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340::writedata(uint8_t c) {
  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *clkport &= ~clkpinmask;
  //digitalWrite(_sclk, LOW);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);
  
  spiwrite(c);

  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
} 


// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
/*
PROGMEM static prog_uchar
  BeginCMD[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ILI9340_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ILI9340_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ILI9340_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ILI9340_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ILI9340_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ILI9340_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ILI9340_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ILI9340_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ILI9340_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ILI9340_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ILI9340_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ILI9340_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ILI9340_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ILI9340_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ILI9340_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ILI9340_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ILI9340_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ILI9340_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ILI9340_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ILI9340_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ILI9340_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ILI9340_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ILI9340_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ILI9340_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ILI9340_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ILI9340_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ILI9340_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ILI9340_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ILI9340_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ILI9340_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ILI9340_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ILI9340_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ILI9340_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ILI9340_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ILI9340_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ILI9340_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ILI9340_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ILI9340_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ILI9340_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ILI9340_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ILI9340_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay
*/

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

  constructor(ILI9340_TFTWIDTH, ILI9340_TFTHEIGHT);

  pinMode(_rst, OUTPUT);
  digitalWrite(_rst, LOW);
  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);

  if(hwSPI) { // Using hardware SPI
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
  }

  // toggle RST low to reset

  digitalWrite(_rst, HIGH);
  delay(5);
  digitalWrite(_rst, LOW);
  delay(20);
  digitalWrite(_rst, HIGH);
  delay(150);

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
  writedata(0x3e); //对比度调节
  writedata(0x28); 
  
  writecommand(ILI9340_VMCTR2);    //VCM control2 
  writedata(0x86);  //--
 
  writecommand(ILI9340_MADCTL);    // Memory Access Control 
  writedata(0x48);

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
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}

void Adafruit_ILI9340::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
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

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
}


void Adafruit_ILI9340::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

  uint8_t hi = color >> 8, lo = color;
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  //digitalWrite(_dc, HIGH);
  //digitalWrite(_cs, LOW);
  while (w--) {
    spiwrite(hi);
    spiwrite(lo);
  }
  *csport |= cspinmask;
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

  *dcport |=  dcpinmask;
  //digitalWrite(_dc, HIGH);
  *csport &= ~cspinmask;
  //digitalWrite(_cs, LOW);

  for(y=h; y>0; y--) {
    for(x=w; x>0; x--) {
      spiwrite(hi);
      spiwrite(lo);
    }
  }
  //digitalWrite(_cs, HIGH);
  *csport |= cspinmask;
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9340::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9340::setRotation(uint8_t m) {

  writecommand(ILI9340_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
     break;
   case 1:
     writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
     _width  = ILI9340_TFTHEIGHT;
     _height = ILI9340_TFTWIDTH;
     break;
  case 2:
    writedata(MADCTL_RGB);
     _width  = ILI9340_TFTWIDTH;
     _height = ILI9340_TFTHEIGHT;
    break;
   case 3:
     writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
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
    SPDR = 0x00;
    while(!(SPSR & _BV(SPIF)));
    r = SPDR;
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
