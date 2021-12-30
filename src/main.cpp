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

float p = 3.1415926;

void testdrawtext(char *text, uint16_t color) {
    tft.setCursor(0,0);
    tft.setTextColor(color);
    tft.print(text);
}

char line = 0;
void printLn(char *text) {
    tft.setCursor(0, line);
    tft.print(text);
    line += 12;
}

void setup(void) {
    Serial.begin(9600);
    Serial.print("hello!");
    pinMode(10, OUTPUT);
    tft.begin();

    Serial.println("init");

    // You can optionally rotate the display by running the line below.
    // Note that a value of 0 means no rotation, 1 means 90 clockwise,
    // 2 means 180 degrees clockwise, and 3 means 270 degrees clockwise.
    //tft.setRotation(1);
    // NOTE: The test pattern at the start will NOT be rotated!  The code
    // for rendering the test pattern talks directly to the display and
    // ignores any rotation.

    uint16_t time = millis();
    tft.fillRect(0, 0, 128, 128, BLACK);
    time = millis() - time;

    Serial.println(time, DEC);

    tft.setTextColor(GREEN);
    printLn("Artificial Horizon v0.9");

    if (mpu.begin()) {
        hasMpu = 1;
        mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
        mpu.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
        printLn("MPU initialised.");
    } else {
        testdrawtext("Cannot initialise MPU", RED);
    }
    delay(500);
     tft.fillScreen(BLACK);
}

float v[7] = { 0, 0, 0, 0, 0, 0, 0 };
//char first = 1;

void printValue(float value, int index) {
    int y = index * 12;
//    if (!first) {
//        tft.setTextColor(BLACK);
//        tft.setCursor(0, y);
//        tft.print(v[index]);
//    }
    tft.setTextColor(index < 3 ? WHITE : index < 6 ? YELLOW : CYAN);
    tft.setCursor(0, y);
    tft.print(value);
    v[index] = value;
}

void testMpu6050() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

//    printValue(a.acceleration.x, 0);
//    printValue(a.acceleration.y, 1);
//    printValue(a.acceleration.z, 2);
//
//    printValue(g.gyro.x, 3);
//    printValue(g.gyro.y, 4);
//    printValue(g.gyro.z, 5);
//
//    printValue(temp.temperature, 6);
//    first = 0;
}

// We want to display numbers, and we want to do that very fast.
// Rather than building a RAM buffer - we don't have much RAM -
// we can hardcode our digit renderers. And rather than overlaying
// it, and double-rendering and introducing flicker, it's far quicker
// to do all rendering in one pass. Without memory for a frame buffer,
// we therefore need to do this in our hardcoded write function.
// Either that, or we use 10% of the chip's memory as a row or column
// buffer, and write only to that.
char characters[6*7] = {};
const char char0 = 0;
const char char1 = 1;
const char char2 = 2;
const char char3 = 3;
const char char4 = 4;
const char char5 = 5;
const char char6 = 6;
const char char7 = 7;
const char char8 = 8;
const char char9 = 9;
const char charNeg = 10;
const char charDot = 11;
const char charSpace = 15;

void printToBuffer(char row, char colourIndex, float value) {
    char colourBits = colourIndex << 4;
    char offset = row * 7;
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

// 16bit colours are 5 bits red, 6 bits green, 5 bits blue
// 116, 203, 255
const uint16_t blue = 0x733F; // 01110 011001 11111
// 179, 121, 41
const uint16_t brown = 0x63C5; // 01100 011110 00101

char overlayBuffer[16] = {};
char brownBuffer[16] = {};

// This is horribly slow
void drawHorizonVLineRunLength(int16_t x, int16_t yintercept) {
    // create Horizon layer
    uint8_t y = 0;
    if (yintercept < 0) {
        for (int i = 0; i < 16; i++) {
            brownBuffer[i] = 0xFF;
        }
    } else if (yintercept > 127) {
        for (int i = 0; i < 16; i++) {
            brownBuffer[i] = 0x00;
        }
    } else {
        // Which block should we bash bits in?
        // yintercept   block   bit_start   bits
        // 0            0       1           FF
        // 1            0       2           FE
        // 2            0       4           FC
        // 7            0       128         1
        // 8            1       0           FF
        // 9            1       1           FE
        const uint8_t yintercept_block = yintercept >> 3;
        y = 0;
        while (y < yintercept_block) {
            brownBuffer[y++] = 0;
        }
        // FF >> 0 == FF, FF >> 7 ==
        const uint8_t shift = yintercept & 0x07;
        brownBuffer[y++] = 0xFF << shift;
        while (y < 16) {
            brownBuffer[y++] = 0xFF;
        }
    }

    // Create character layer
    if (x < 32) {
        y = 0;
        while (y < 4) {
            overlayBuffer[y++] = 0x11;
        }
        while (y < 16) {
            overlayBuffer[y++] = 0x00;
        }
    }

    // Combine layers into a set of pixel operations
    uint16_t runColour = blue;
    int16_t runStart = 0;
    uint8_t runLength = 0;
    for(y = 0; y < 128; y++) {
        const uint8_t block = y >> 3;
        const uint8_t bit = 1 << (y & 0x07);
        uint16_t c = blue;
        if (brownBuffer[block] & bit) {
            c = brown;
        }
        if (overlayBuffer[block] & bit) {
            c = WHITE;
        }
        if (c == runColour) {
            runLength++;
        } else {
            // Output the last run
            if (runLength > 0) {
                tft.writeFillRectPreclipped(x, runStart, 1, runLength, runColour);
                //tft.drawFastVLine(x, runStart, runLength, runColour);
            }
            // Start a new one.
            runColour = c;
            runStart = y;
            runLength = 1;
        }
    }
    // Output the last run.
    tft.writeFillRectPreclipped(x, runStart, 1, runLength, runColour);
    //tft.drawFastVLine(x, runStart, runLength, runColour);
}

uint16_t colourBuffer[16] = {};

// This is relatively fast.
void drawHorizonVWritePixels(int16_t x, int16_t yintercept) {
    for (int band = 0; band < 8; band++) {
        int ystart = band * 16;
        int yend = ystart + 16;
        tft.setAddrWindow(x, ystart, 1, 16);
        int y = 0;
        while (y < yintercept && y < 16) {
            colourBuffer[y++] = blue;
        }
        while (y >= yintercept && y < 16) {
            colourBuffer[y++] = brown;
        }
        yintercept -= 16;
        // Overlay "text"
        if (band == 0) {
            colourBuffer[0] = WHITE;
            colourBuffer[4] = WHITE;
            colourBuffer[8] = WHITE;
            colourBuffer[12] = WHITE;
        }
        tft.writePixels(colourBuffer, 16, true, false);
    }
}

int16_t test = 0;

void drawHorizon() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    printToBuffer(0, 0, a.acceleration.x);
    printToBuffer(1, 0, a.acceleration.y);
    printToBuffer(2, 0, a.acceleration.z);
//    printValue(a.acceleration.x, 0);
//    printValue(a.acceleration.y, 1);
//    printValue(a.acceleration.z, 2);

    // Transaction
    tft.startWrite();

    // Find the equation of a line which expresses the horizon.
    // Using slope-intercept y = mx + c. Note that we will need
    // special handling to cope with 90 degrees, since m will be INF
    float m = 0;
    float c = 0 + test % 128 - 64;
    int16_t y = 0;
    test ++;


    // Draw the horizon - brown below the line, blue above.
    for (int16_t x = 0; x < 128; x++) {
        y = m * x + c + 64;
        // Draw 0-y as blue
//        drawHorizonVLineRunLength(x, y);
        drawHorizonVWritePixels(x, y);

//        if (y > 0) {
//            tft.writeFillRectPreclipped(x, 0, 1, y, blue);
////            tft.writeFastVLine(x, 0, y, blue);
//        }
//        if (y < 128) {
//            tft.writeFillRectPreclipped(x, y, 1, 127-y, brown);
////            tft.writeFastVLine(x, y, 127-y, brown);
//        }
    }

    tft.endWrite();
}

void loop() {
    if (hasMpu == 1) {
        drawHorizon();
        //testMpu6050();
        //delay(100);
    }
}