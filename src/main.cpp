/***************************************************
  This is a example sketch demonstrating graphic drawing
  capabilities of the SSD1351 library for the 1.5"
  and 1.27" 16-bit Color OLEDs with SSD1351 driver chip

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1431
  ------> http://www.adafruit.com/products/1673

  If you're using a 1.27" OLED, change SCREEN_HEIGHT to 96 instead of 128.

  These displays use SPI to communicate, 4 or 5 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution

  The Adafruit GFX Graphics core library is also required
  https://github.com/adafruit/Adafruit-GFX-Library
  Be sure to install it!
 ****************************************************/

#define SCREEN_SIZE  128
#define LAST_SCREEN_INDEX (SCREEN_SIZE - 1)

// You can use any (4 or) 5 pins
#define SCLK_PIN 2
#define MOSI_PIN 3
#define DC_PIN   4
#define CS_PIN   5
#define RST_PIN  6

// Documentation asks us to set this in order to use hardware SPI?
#define SPI_ALT_PIN 10

// Use to trigger the DSO for timing analysis
#define DSO_TEST_PIN 8

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>

Adafruit_MPU6050 mpu;
char hasMpu = 0;

// DC_PIN
uint8_t dcPinMaskSet;
volatile uint8_t *dcPort;
uint8_t csPinMaskSet;
volatile uint8_t *csPort;
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )

const float p = 3.1415926;
const long spiClockSpeedHz = 8000000L;

SPISettings spiSettings = SPISettings(spiClockSpeedHz, MSBFIRST, SPI_MODE0);
SPIClass *spi;

#define SSD1351_CMD_SETCOLUMN 0x15      ///< See datasheet
#define SSD1351_CMD_SETROW 0x75         ///< See datasheet
#define SSD1351_CMD_WRITERAM 0x5C       ///< See datasheet
#define SSD1351_CMD_SETREMAP 0xA0       ///< See datasheet
#define SSD1351_CMD_STARTLINE 0xA1      ///< See datasheet
#define SSD1351_CMD_DISPLAYOFFSET 0xA2  ///< See datasheet
#define SSD1351_CMD_NORMALDISPLAY 0xA6  ///< See datasheet
#define SSD1351_CMD_INVERTDISPLAY 0xA7  ///< See datasheet
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
void spiStartWrite() {
    SPI.beginTransaction(spiSettings);
    *csPort &= (~csPinMaskSet);
}

void spiEndWrite() {
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

typedef struct {
    float x, y, z;
} Vector;

#define vecLengthSq(v) ((v).x*(v).x + (v).y*(v).y + (v).z*(v).z)
#define vecLength(v) (sqrtf(vecLengthSq(v)))
#define dotProduct(v1, v2) ((v1).x*(v2).x + (v1).y*(v2).y + (v1).z*(v2).z)

Vector vecForward = { .x = -1, .y = 0, .z = 0 };

float old_m = 0;
float old_c = 129;
boolean first = true;
uint16_t lastFrame = millis();

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

void setup(void) {
    pinMode(LED_BUILTIN, OUTPUT); // Flash the built-in LED if we failed init.
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(SPI_ALT_PIN, OUTPUT);
    pinMode(DSO_TEST_PIN, OUTPUT);

    dcPinMaskSet = digitalPinToBitMask(DC_PIN);
    dcPort = portOutputRegister(digitalPinToPort(DC_PIN));

    displaySetup();

    if (mpu.begin()) {
        hasMpu = 1;
        mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    } else {
    }
    digitalWrite(LED_BUILTIN, LOW);
}

const char charNeg = 10;
const char charDot = 11;
const char charSpace = 15;

char characters[6*7] = {
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
        charSpace, charSpace,charSpace,charSpace,charSpace,charSpace,
};

void printToBuffer(char row, char colourIndex, float value) {
    char colourBits = colourIndex << 4;
    char offset = row * 6;
    int16_t ivalue = value * 100; // work with ints
    characters[offset + 0] = colourBits | (ivalue < 0 ? charNeg : charSpace);
    if (ivalue < 0) {
        ivalue = -ivalue;
    }
    if (ivalue > 9999) {
        ivalue = ivalue % 10000;
    }
    characters[offset + 5] = colourBits | (ivalue % 10);
    ivalue /= 10;
    characters[offset + 4] = colourBits | (ivalue % 10);
    ivalue /= 10;
    characters[offset + 3] = charDot;
    characters[offset + 2] = colourBits | (ivalue == 0 ? charSpace : ivalue % 10);
    ivalue /= 10;
    characters[offset + 1] = colourBits | (ivalue == 0 ? charSpace : ivalue % 10);
}

const uint16_t X_Blue = 0x3F73;
const uint16_t X_Brown = 0xC563;
const uint16_t X_Yellow = 0xE0FF;
const uint16_t X_White = 0xFFFF;
const uint16_t X_Cyan = 0xFF07;
const uint16_t X_Red = 0x00F8;
const uint16_t X_Green = 0xE007;

uint16_t colourBuffer[SCREEN_SIZE] = {};

//  xx   x    xx  xxx  x x  xxxx  xxx xxxx  xx   xx
// x  x xx   x  x    x x x  x    x       x x  x x  x
// x  x  x     x    x  xxxx xxx  xxx    x   xx   xxx xxxx
// x  x  x    x      x   x     x x  x  x   x  x    x
//  xx  xxx  xxxx xxx    x  xxx   xx  x     xx    x        xx
uint8_t vfont[] = {
        0x0E, 0x11, 0x11, 0x0E, // 0
        0x12, 0x1F, 0x10, 0x00, // 1
        0x12, 0x19, 0x15, 0x12, // 2
        0x11, 0x11, 0x15, 0x0A, // 3
        0x07, 0x04, 0x1F, 0x04, // 4
        0x17, 0x15, 0x15, 0x09, // 5
        0x0E, 0x15, 0x15, 0x09, // 6
        0x11, 0x09, 0x05, 0x03, // 7
        0x0A, 0x15, 0x15, 0x0A, // 8
        0x02, 0x05, 0x15, 0x0E, // 9
        0x04, 0x04, 0x04, 0x04, // -
        0x00, 0x10, 0x10, 0x00, // .
};
uint16_t colours[] = { X_White, X_Cyan, X_Yellow, X_Red };
void blitCharacterVSlice(char packedCharacter, char xSlice, char bufferOffset) {
    char character = packedCharacter & 0x0F;
    if (character == charSpace || xSlice == 4)
        return;
    uint16_t c = colours[packedCharacter >> 4];
    char slice = character * 4 + xSlice;
    uint16_t *p = &colourBuffer[bufferOffset];
    uint8_t fontslice = vfont[slice];
    for (char i = 0; i < 5; i++) {
        if (fontslice & 0x01) {
            p[0] = c;
        }
        if (fontslice & 0x02) {
            p[1] = c;
        }
        if (fontslice & 0x04) {
            p[2] = c;
        }
        if (fontslice & 0x08) {
            p[3] = c;
        }
        if (fontslice & 0x10) {
            p[4] = c;
        }
    }
}

// Overlay --v-- shape
char target[] = { 64, 64, 64, 64, 64, 64, 65, 66, 67, 66, 65, 64, 64, 64, 64, 64, 64 };

// A loop-unrollled bulk paint, since the loop is actually significant
void _writePixels(uint16_t *colors, uint16_t y1, uint16_t y2) {
    uint8_t *buf = (uint8_t *) colors + (y1 << 1);
    uint8_t *end = (uint8_t *) colors + ((y2 + 1) << 1);
    while (buf < end) {
        AVR_WRITESPI(*buf++);
        AVR_WRITESPI(*buf++);
    }
}

void _setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    spiWriteCommand(SSD1351_CMD_SETCOLUMN); // X range
    AVR_WRITESPI(x1);
    AVR_WRITESPI(x2);
    spiWriteCommand(SSD1351_CMD_SETROW); // Y range
    AVR_WRITESPI(y1);
    AVR_WRITESPI(y2);
    spiWriteCommand(SSD1351_CMD_WRITERAM); // Begin write
}

// This is relatively fast.
void drawHorizonVWritePixels(int16_t x, int16_t yintercept, int16_t old_yintercept, bool clear) {
    bool hasCharsX = x < 6 * 5;
    char charCol = x / 5;
    char xBit = x % 5;
    int y = 0; // start at the top if rewriting the characters
    int16_t y1 = 0;
    int16_t y2 = LAST_SCREEN_INDEX;
    if (!clear) {
        y1 = min(LAST_SCREEN_INDEX, max(0, yintercept < old_yintercept ? yintercept : old_yintercept));
        y2 = min(LAST_SCREEN_INDEX, max(0, yintercept < old_yintercept ? old_yintercept : yintercept));
        if (hasCharsX && xBit != 4) {
            y1 = 0;
        }
    }
    _setAddrWindow(x, y1, x, y2);
    if (yintercept < 0) {
        while (y < SCREEN_SIZE) {
            colourBuffer[y++] = X_Brown;
        }
    } else if (yintercept > LAST_SCREEN_INDEX) {
        while (y < SCREEN_SIZE) {
            colourBuffer[y++] = X_Blue;
        }
    } else {
        while (y < yintercept) {
            colourBuffer[y++] = X_Blue;
        }
        while (y < SCREEN_SIZE) {
            colourBuffer[y++] = X_Brown;
        }
    }

    if (hasCharsX) {
        for (int charRow = 0; charRow < 7; charRow++) {
            char charIndex = charCol + charRow * 6;
            char character = characters[charIndex];
            blitCharacterVSlice(character, xBit, charRow << 3);
        }
    }

    if (x >= 56 && x <= 72) {
        colourBuffer[target[x - 56]] = X_Yellow;
    }

    _writePixels(colourBuffer, y1, y2);
}

void vecNormalise(Vector &v) {
    float length = vecLength(v);
    v.x /= length;
    v.y /= length;
    v.z /= length;
}

Vector vecCrossProduct(Vector &v1, Vector &v2) {
    Vector result = {};
    result.x = v1.y*v2.z - v1.z*v2.y;
    result.y = v1.z*v2.x - v1.x*v2.z;
    result.z = v1.x*v2.y - v1.y*v2.x;
    return result;
}

// See https://create.arduino.cc/projecthub/MinukaThesathYapa/arduino-mpu6050-accelerometer-f92d8b
// For alternate MPU6050 code.
void drawHorizon() {
    digitalWrite(DSO_TEST_PIN, LOW);
    long start = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // printToBuffer(3, 1, g.gyro.pitch);
    printToBuffer(0, 0, a.acceleration.x);
    printToBuffer(1, 0, a.acceleration.y);
    printToBuffer(2, 0, a.acceleration.z);
    printToBuffer(6, 2, temp.temperature);

    uint16_t now = millis();
    float fps = 1000.0f / (now - lastFrame); // (now - lastFrame) / 100.0f; //
    printToBuffer(5, 1, fps);
    lastFrame = now;

    // Transaction
    spiStartWrite();

    // Build a vector to "down", and normalise it, ie 0,0,1
    // given our forward vector is -1,0,0 (depends on sensor orientation),
    // cross product of forward and down should correlate with horizon slope?
    // and angle between forward and down gives horizon offset?
    // dot product of forward and down gives us the cos of the angle between them
    Vector accel = { .x = a.acceleration.x, .y = a.acceleration.y, .z = a.acceleration.z };
    vecNormalise(accel);
    float cosPitch = dotProduct(accel, vecForward);
    Vector vecSide = vecCrossProduct(accel, vecForward);
    float m = vecSide.z / vecSide.y;

    // Find the equation of a line which expresses the horizon.
    // Using slope-intercept y = mx + c. Note that we will need
    // special handling to cope with 90 degrees, since m will be INF
    float c = -70 * cosPitch + 64 - m * 64; // the m*64 is so we don't have to m*(x-64) each loop

    // We write the first column separately for two reasons.
    // First, we can completely avoid a pair of unnecessary multiplies and additions.
    // Second, if we trigger the DSO after the first column it makes some measurements a bit easier.
    drawHorizonVWritePixels(0, c, old_c, first);

    digitalWrite(DSO_TEST_PIN, HIGH);
    // Draw the horizon - brown below the line, blue above.
    for (int16_t x = 1; x < SCREEN_SIZE; x++) {
        drawHorizonVWritePixels(x, m * x + c, old_m * x + old_c, first);
    }

    old_m = m;
    old_c = c;
    first = false;

    spiEndWrite();
}

void loop() {
    if (hasMpu == 1) {
        drawHorizon();
    } else {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
}