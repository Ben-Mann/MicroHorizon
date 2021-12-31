/**
 * Fast SSD1351 Interface for the ArduinoNano / ATmega328P on hardware SPI
 */

#include <SPI.h>
#include "display.h"

#define DC_PIN   4
#define CS_PIN   5
#define RST_PIN  6

// Documentation asks us to set this in order to use hardware SPI?
#define SPI_ALT_PIN 10
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )

uint8_t dcPinMaskSet;
volatile uint8_t *dcPort;
uint8_t csPinMaskSet;
volatile uint8_t *csPort;
const long spiClockSpeedHz = 8000000L;

SPISettings spiSettings = SPISettings(spiClockSpeedHz, MSBFIRST, SPI_MODE0);

#define SSD1351_CMD_SETCOLUMN 0x15      ///< See datasheet
#define SSD1351_CMD_SETROW 0x75         ///< See datasheet
#define SSD1351_CMD_WRITERAM 0x5C       ///< See datasheet
#define SSD1351_CMD_SETREMAP 0xA0       ///< See datasheet
#define SSD1351_CMD_STARTLINE 0xA1      ///< See datasheet
#define SSD1351_CMD_DISPLAYOFFSET 0xA2  ///< See datasheet
#define SSD1351_CMD_NORMALDISPLAY 0xA6  ///< See datasheet
#define SSD1351_CMD_FUNCTIONSELECT 0xAB ///< See datasheet
#define SSD1351_CMD_DISPLAYOFF 0xAE     ///< See datasheet
#define SSD1351_CMD_DISPLAYON 0xAF      ///< See datasheet
#define SSD1351_CMD_PRECHARGE 0xB1      ///< See datasheet
#define SSD1351_CMD_CLOCKDIV 0xB3       ///< See datasheet
#define SSD1351_CMD_SETVSL 0xB4         ///< See datasheet
#define SSD1351_CMD_SETGPIO 0xB5        ///< See datasheet
#define SSD1351_CMD_PRECHARGE2 0xB6     ///< See datasheet
#define SSD1351_CMD_VCOMH 0xBE          ///< See datasheet
#define SSD1351_CMD_CONTRASTABC 0xC1    ///< See datasheet
#define SSD1351_CMD_CONTRASTMASTER 0xC7 ///< See datasheet
#define SSD1351_CMD_MUXRATIO 0xCA       ///< See datasheet
#define SSD1351_CMD_COMMANDLOCK 0xFD    ///< See datasheet

// stuff a byte into the SPI data register, and then poll until it gets shifted out
#define AVR_WRITESPI(x) for (SPDR = (x); (!(SPSR & _BV(SPIF)));)
#define SPI_DC_LOW() *dcPort &= ~(dcPinMaskSet)
#define SPI_DC_HIGH() *dcPort |= dcPinMaskSet
#define SPI_CS_LOW() *csPort &= (~csPinMaskSet)
#define SPI_CS_HIGH() *csPort |= csPinMaskSet

// Not using chip select
void displayStartWrite() {
    SPI.beginTransaction(spiSettings);
    *csPort &= (~csPinMaskSet);
}

void displayEndWrite() {
    *csPort |= csPinMaskSet;
    SPI.endTransaction();
}

void spiWriteCommand(uint8_t cmd) {
    SPI_DC_LOW();
    AVR_WRITESPI(cmd);
    SPI_DC_HIGH();
}

void spiWriteProgMemCommand(uint8_t commandByte, const uint8_t *dataBytes, uint8_t numDataBytes) {
    SPI.beginTransaction(spiSettings);
    SPI_CS_LOW();

    SPI_DC_LOW();          // Command mode
    AVR_WRITESPI(commandByte); // Send the command byte

    SPI_DC_HIGH();
    for (int i = 0; i < numDataBytes; i++) {
        AVR_WRITESPI(pgm_read_byte(dataBytes++));
    }

    SPI_CS_HIGH();
    SPI.endTransaction();
}

// See Adafruit_SSD1351 for a reference example
static const uint8_t PROGMEM lcdBoot[] = {
        SSD1351_CMD_COMMANDLOCK,    1, 0x12,
        SSD1351_CMD_COMMANDLOCK,    1, 0xB1,
        SSD1351_CMD_DISPLAYOFF,     0,
        SSD1351_CMD_CLOCKDIV,       1, 0xF1, // 7:4 = Oscillator Freq, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
        SSD1351_CMD_MUXRATIO,       1, 127,
        SSD1351_CMD_DISPLAYOFFSET,  1, 0x0,
        SSD1351_CMD_SETGPIO,        1, 0x00,
        SSD1351_CMD_FUNCTIONSELECT, 1, 0x01, // internal (diode drop)
        SSD1351_CMD_PRECHARGE,      1, 0x32,
        SSD1351_CMD_VCOMH,          1, 0x05,
        SSD1351_CMD_NORMALDISPLAY,  0,
        SSD1351_CMD_CONTRASTABC,    3, 0xC8, 0x80, 0xC8,
        SSD1351_CMD_CONTRASTMASTER, 1, 0x0F,
        SSD1351_CMD_SETVSL,         3, 0xA0, 0xB5, 0x55,
        SSD1351_CMD_PRECHARGE2,     1, 0x01,
        SSD1351_CMD_DISPLAYON,      0,  // Main screen turn on
        SSD1351_CMD_SETREMAP,       1, 0b01100100 | 0b00010000,
        SSD1351_CMD_STARTLINE,      1, SCREEN_SIZE,
        0}; // END OF COMMAND LIST

void displaySetup() {
    pinMode(SPI_ALT_PIN, OUTPUT);
    dcPinMaskSet = digitalPinToBitMask(DC_PIN);
    dcPort = portOutputRegister(digitalPinToPort(DC_PIN));
    csPinMaskSet = digitalPinToBitMask(CS_PIN);
    csPort = portOutputRegister(digitalPinToPort(CS_PIN));

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    pinMode(DC_PIN, OUTPUT);
    digitalWrite(DC_PIN, HIGH); // Data mode

    SPI.begin();

    pinMode(RST_PIN, OUTPUT);
    digitalWrite(RST_PIN, HIGH);
    delay(100);
    digitalWrite(RST_PIN, LOW);
    delay(100);
    digitalWrite(RST_PIN, HIGH);
    delay(200);

    const uint8_t *addr = (const uint8_t *)lcdBoot;
    uint8_t cmd, countFlag, numArgs;
    while ((cmd = pgm_read_byte(addr++)) > 0) { // '0' command ends list
        countFlag = pgm_read_byte(addr++);
        numArgs = countFlag & 0x7F;
        if (cmd != 0xFF) { // '255' is ignored
            spiWriteProgMemCommand(cmd, addr, numArgs);
        }
        addr += numArgs;
    }
}

// A loop-unrollled bulk paint, since the loop is actually significant
void displayWritePixels(uint16_t *colors, uint16_t y1, uint16_t y2) {
    uint8_t *buf = (uint8_t *) colors + (y1 << 1);
    uint8_t *end = (uint8_t *) colors + ((y2 + 1) << 1);
    while (buf < end) {
        AVR_WRITESPI(*buf++);
        AVR_WRITESPI(*buf++);
    }
}

void displaySetAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    spiWriteCommand(SSD1351_CMD_SETCOLUMN); // X range
    AVR_WRITESPI(x1);
    AVR_WRITESPI(x2);
    spiWriteCommand(SSD1351_CMD_SETROW); // Y range
    AVR_WRITESPI(y1);
    AVR_WRITESPI(y2);
    spiWriteCommand(SSD1351_CMD_WRITERAM); // Begin write
}