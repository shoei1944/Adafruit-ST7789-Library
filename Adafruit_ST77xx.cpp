#include "Adafruit_ST77xx.h"
#include <limits.h>
// #include "pins_arduino.h"
// #include "wiring_private.h"
#include <SPI.h>

#define SPI_DEFAULT_FREQ 38000000 ///< Default SPI data clock frequency

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST77XX driver with software SPI
    @param  w     Display width in pixels at default rotation setting (0)
    @param  h     Display height in pixels at default rotation setting (0)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  mosi  SPI MOSI pin #
    @param  sclk  SPI Clock pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
    @param  miso  SPI MISO pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_ST77xx::Adafruit_ST77xx(uint16_t w, uint16_t h, int8_t cs, int8_t dc,
                                 int8_t mosi, int8_t sclk, int8_t rst,
                                 int8_t miso)
    : Adafruit_SPITFT(w, h, cs, dc, mosi, sclk, rst, miso) {}

/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST77XX driver with hardware SPI
    @param  w     Display width in pixels at default rotation setting (0)
    @param  h     Display height in pixels at default rotation setting (0)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_ST77xx::Adafruit_ST77xx(uint16_t w, uint16_t h, int8_t cs, int8_t dc,
                                 int8_t rst)
    : Adafruit_SPITFT(w, h, cs, dc, rst) {}

#if !defined(ESP8266)
/**************************************************************************/
/*!
    @brief  Instantiate Adafruit ST77XX driver with selectable hardware SPI
    @param  w     Display width in pixels at default rotation setting (0)
    @param  h     Display height in pixels at default rotation setting (0)
    @param  spiClass A pointer to an SPI device to use (e.g. &SPI1)
    @param  cs    Chip select pin #
    @param  dc    Data/Command pin #
    @param  rst   Reset pin # (optional, pass -1 if unused)
*/
/**************************************************************************/
Adafruit_ST77xx::Adafruit_ST77xx(uint16_t w, uint16_t h, SPIClass *spiClass,
                                 int8_t cs, int8_t dc, int8_t rst)
    : Adafruit_SPITFT(w, h, spiClass, cs, dc, rst) {}
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Companion code to the initiliazation tables. Reads and issues
            a series of LCD commands stored in PROGMEM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_ST77xx::displayInit(const uint8_t *addr) {

  uint8_t numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++); // Number of commands to follow
  while (numCommands--) {              // For each command...
    cmd = pgm_read_byte(addr++);       // Read command
    numArgs = pgm_read_byte(addr++);   // Number of args to follow
    ms = numArgs & ST_CMD_DELAY;       // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;          // Mask out delay bit
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;

    if (ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if (ms == 255)
        ms = 500; // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

/**************************************************************************/
/*!
    @brief  Initialize ST77xx chip. Connects to the ST77XX over SPI and
            sends initialization procedure commands
    @param  freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_ST77xx::begin(uint32_t freq) {
  if (!freq) {
    freq = SPI_DEFAULT_FREQ;
  }
  _freq = freq;

  invertOnCommand = ST77XX_INVON;
  invertOffCommand = ST77XX_INVOFF;

  initSPI(freq, spiMode);
}

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST77XX displays
    @param  cmdList  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_ST77xx::commonInit(const uint8_t *cmdList) {
  begin();

  if (cmdList) {
    displayInit(cmdList);
  }
}

/**************************************************************************/
/*!
  @brief  SPI displays set an address window rectangle for blitting pixels
  @param  x  Top left corner x coordinate
  @param  y  Top left corner y coordinate
  @param  w  Width of window
  @param  h  Height of window
*/
/**************************************************************************/
void Adafruit_ST77xx::setAddrWindow(uint16_t x, uint16_t y, uint16_t w,
                                    uint16_t h) {
  x += _xstart;
  y += _ystart;
  uint32_t xa = ((uint32_t)x << 16) | (x + w - 1);
  uint32_t ya = ((uint32_t)y << 16) | (y + h - 1);

  writeCommand(ST77XX_CASET); // Column addr set
  SPI_WRITE32(xa);

  writeCommand(ST77XX_RASET); // Row addr set
  SPI_WRITE32(ya);

  writeCommand(ST77XX_RAMWR); // write to RAM
}

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) and orientation of TFT display
    @param  m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_ST77xx::setRotation(uint8_t m) {
  uint8_t madctl = 0;

  rotation = m % 4; // can't be higher than 3

  switch (rotation) {
  case 0:
    madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
    _xstart = _colstart;
    _ystart = _rowstart;
    break;
  case 1:
    madctl = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
    _ystart = _colstart;
    _xstart = _rowstart;
    break;
  case 2:
    madctl = ST77XX_MADCTL_RGB;
    _xstart = _colstart;
    _ystart = _rowstart;
    break;
  case 3:
    madctl = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
    _ystart = _colstart;
    _xstart = _rowstart;
    break;
  }

  sendCommand(ST77XX_MADCTL, &madctl, 1);
}

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) of display with offsets
    @param  col  The offset from 0 for the column address
    @param  row  The offset from 0 for the row address
*/
/**************************************************************************/
void Adafruit_ST77xx::setColRowStart(int8_t col, int8_t row) {
  _colstart = col;
  _rowstart = row;
}

/**************************************************************************/
/*!
 @brief  Change whether display is on or off
 @param  enable True if you want the display ON, false OFF
 */
/**************************************************************************/
void Adafruit_ST77xx::enableDisplay(boolean enable) {
  sendCommand(enable ? ST77XX_DISPON : ST77XX_DISPOFF);
}

/**************************************************************************/
/*!
 @brief  Change whether TE pin output is on or off
 @param  enable True if you want the TE pin ON, false OFF
 */
/**************************************************************************/
void Adafruit_ST77xx::enableTearing(boolean enable) {
  sendCommand(enable ? ST77XX_TEON : ST77XX_TEOFF);
}

/**************************************************************************/
/*!
 @brief  Change whether sleep mode is on or off
 @param  enable True if you want sleep mode ON, false OFF
 */
/**************************************************************************/
void Adafruit_ST77xx::enableSleep(boolean enable) {
  sendCommand(enable ? ST77XX_SLPIN : ST77XX_SLPOUT);
}