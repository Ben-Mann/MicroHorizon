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

// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

// You can use any (4 or) 5 pins
#define SCLK_PIN 2
#define MOSI_PIN 3
#define DC_PIN   4
#define CS_PIN   5
#define RST_PIN  6

// Color definitions
#define	BLACK           0x0000
#define	BLUE            0x001F
#define	RED             0xF800
#define	GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define YELLOW          0xFFE0
#define WHITE           0xFFFF

#define RUN_TEST_PATTERNS 0

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>


// Option 1: use any pins but a little slower
//Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, CS_PIN, DC_PIN, MOSI_PIN, SCLK_PIN, RST_PIN);

// Option 2: must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_SSD1351 tft = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

Adafruit_MPU6050 mpu;
char hasMpu = 0;

// DC_PIN
uint8_t dcPinMaskSet;
volatile uint8_t *dcPort;
#define digitalPinToPort(P) ( pgm_read_byte( digital_pin_to_port_PGM + (P) ) )

const float p = 3.1415926;

void setup(void) {
    // SPI.setClockDivider(SPI_CLOCK_DIV2); // Does nothing, it's already at max freq.
    pinMode(LED_BUILTIN, OUTPUT); // Flash the built-in LED if we failed init.
    digitalWrite(LED_BUILTIN, HIGH);
    pinMode(10, OUTPUT);
    pinMode(8, OUTPUT);
    tft.begin();

    dcPinMaskSet = digitalPinToBitMask(DC_PIN);
    dcPort = portOutputRegister(digitalPinToPort(DC_PIN));

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

uint16_t colourBuffer[128] = {};

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

// stuff a byte into the SPI data register, and then poll until it gets shifted out
#define AVR_WRITESPI(x) for (SPDR = (x); (!(SPSR & _BV(SPIF)));)

// A loop-unrollled bulk paint, since the loop is actually significant
void _writePixels(uint16_t *colors, uint16_t y1, uint16_t y2) {
    uint8_t *buf = (uint8_t *) colors + (y1 << 1);
    uint8_t *end = (uint8_t *) colors + (y2 << 1);
    while (buf < end) {
        AVR_WRITESPI(*buf++);
        AVR_WRITESPI(*buf++);
    }
}

#define SPI_DC_LOW() *dcPort &= ~(dcPinMaskSet)
#define SPI_DC_HIGH() *dcPort |= dcPinMaskSet

void _writeCommand(uint8_t cmd) {
    SPI_DC_LOW();
    AVR_WRITESPI(cmd);
    SPI_DC_HIGH();
}

// The tft version asks for w and h, so must compute this every frame, but
// we're always sending some things the same, so we just want x1=x2, y1=0,y2=128
void _setAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    _writeCommand(SSD1351_CMD_SETCOLUMN); // X range
    AVR_WRITESPI(x1);
    AVR_WRITESPI(x2);
    _writeCommand(SSD1351_CMD_SETROW); // Y range
    AVR_WRITESPI(y1);
    AVR_WRITESPI(y2);
    _writeCommand(SSD1351_CMD_WRITERAM); // Begin write
}

// This is relatively fast.
void drawHorizonVWritePixels(int16_t x, int16_t yintercept, int16_t old_yintercept) {
    bool hasCharsX = x < 6 * 5;
    char charCol = x / 5;
    char xBit = x % 5;
    int y = 0; // start at the top if rewriting the characters
    int16_t y1 = 0;
    int16_t y2 = min(127, max(0, yintercept < old_yintercept ? old_yintercept : yintercept));
    if (!hasCharsX || xBit ==4) {
        y1 = min(127, max(0, yintercept < old_yintercept ? yintercept : old_yintercept));
    }
    _setAddrWindow(x, y1, x, y2);
    if (yintercept < 0) {
        while (y < 128) {
            colourBuffer[y++] = X_Brown;
        }
    } else if (yintercept > 127) {
        while (y < 128) {
            colourBuffer[y++] = X_Blue;
        }
    } else {
        while (y < yintercept) {
            colourBuffer[y++] = X_Blue;
        }
        while (y < 128) {
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

int16_t test = 0;
uint16_t lastFrame = millis();

typedef struct {
    float x, y, z;
} Vector;

#define vecLengthSq(v) ((v).x*(v).x + (v).y*(v).y + (v).z*(v).z)
#define vecLength(v) (sqrtf(vecLengthSq(v)))

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

#define dotProduct(v1, v2) ((v1).x*(v2).x + (v1).y*(v2).y + (v1).z*(v2).z)

Vector vecForward = { .x = -1, .y = 0, .z = 0 };

float old_m = 0;
float old_c = 129;

// See https://create.arduino.cc/projecthub/MinukaThesathYapa/arduino-mpu6050-accelerometer-f92d8b
// For alternate MPU6050 code.
long duration = 0;
void drawHorizon() {
    digitalWrite(8, LOW);
    long start = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    long mpuTime = millis();

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
    tft.startWrite();

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

    long computeTime = millis();

    printToBuffer(3, 1, (computeTime - start) / 100.0f);
    printToBuffer(4, 1, (duration) / 100.0f);

    drawHorizonVWritePixels(0, c, old_c);

    digitalWrite(8, HIGH);
    // Draw the horizon - brown below the line, blue above.
    for (int16_t x = 1; x < 128; x++) {
        drawHorizonVWritePixels(x, m * x + c, old_m * x + old_c);
    }

    old_m = m;
    old_c = c;

    duration = millis() - computeTime;

    tft.endWrite();
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