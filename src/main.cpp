/**
 * Draw an 'aritificial horizon' display on an SSD1351 128x128 LCD with data from an MPU6050 IMU.
 *
 * The top left of the display shows debug data, nominally:
 *  * X acceleration
 *  * Y acceleration
 *  * Z acceleration
 *  * Refresh rate in Frames Per Second
 *  * IMU temperature
 *
 * TODO - Compensate for gyro rotation.
 *   The IMU gyro is not used, and since the current implementation relies on the orientation of
 *   force vectors with die alignment, linear forces will register as rotation.
 */

#include "display.h"
#include "vector.h"
#include "mpu.h"

// For development. Use to trigger a DSO for timing analysis
#define DSO_TEST_PIN 8

const char CHAR_NEG = 10;
const char CHAR_DOT = 11;
const char CHAR_SPACE = 15;
const uint8_t CHAR_COLS = 6;
const uint8_t CHAR_ROWS = 7;
const uint8_t CHAR_CELLS = CHAR_COLS * CHAR_ROWS;
const uint16_t X_Blue = 0x3F73;
const uint16_t X_Brown = 0xC563;
const uint16_t X_Yellow = 0xE0FF;
const uint16_t X_White = 0xFFFF;
const uint16_t X_Cyan = 0xFF07;
const uint16_t X_Red = 0x00F8;

Vector vecForward = { .x = -1, .y = 0, .z = 0 };
float old_m = 0;
float old_c = 129;
boolean firstFrame = true;
uint16_t lastFrame = millis();
char characters[CHAR_CELLS] = {};
uint16_t colourBuffer[SCREEN_SIZE] = {};

/**
 * Vertical slices of some font digits, for quick blitting.
 *
 * \code
 *  xx   x    xx  xxx  x x  xxxx  xxx xxxx  xx   xx
 * x  x xx   x  x    x x x  x    x       x x  x x  x
 * x  x  x     x    x  xxxx xxx  xxx    x   xx   xxx xxxx
 * x  x  x    x      x   x     x x  x  x   x  x    x
 *  xx  xxx  xxxx xxx    x  xxx   xx  x     xx    x        xx
 * \endcode
 */
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

/** Supported text colours. There's 4 bits available, but don't need that many colours. */
uint16_t colours[] = { X_White, X_Cyan, X_Yellow, X_Red };

/** The heading indicator, a --v-- shape overlay */
char target[] = { 64, 64, 64, 64, 64, 64, 65, 66, 67, 66, 65, 64, 64, 64, 64, 64, 64 };

/**
 * Push a floating value in the range -99.99 to +99.99 and its desired colour into a
 * character grid which we'll later use to paint glyphs.
 */
void printToBuffer(char row, char colourIndex, float value) {
    char colourBits = colourIndex << 4;
    char offset = row * CHAR_COLS;
    int16_t ivalue = value * 100; // work with ints
    characters[offset + 0] = colourBits | (ivalue < 0 ? CHAR_NEG : CHAR_SPACE);
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
    characters[offset + 3] = CHAR_DOT;
    characters[offset + 2] = colourBits | (ivalue == 0 ? CHAR_SPACE : ivalue % 10);
    ivalue /= 10;
    characters[offset + 1] = colourBits | (ivalue == 0 ? CHAR_SPACE : ivalue % 10);
}

/**
 * Transfer a vertical slice of a text character into the vertical painting buffer.
 */
void blitCharacterVSlice(char packedCharacter, char xSlice, char bufferOffset) {
    char character = packedCharacter & 0x0F;
    if (character == CHAR_SPACE || xSlice == 4)
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

/**
 * Paint differences between the last frame and this one along a pixel-wide column of the display.
 * There is no memory for a full frame buffer, but we can manage the RAM for a single column buffer.
 * So we just compare the horizon intercepts, and paint between them - unless we are also painting
 * characters, in which case we repaint them entirely, rather than spending time determining which
 * pixels changed.
 */
void drawHorizonColumn(int16_t x, int16_t yintercept, int16_t old_yintercept, bool clear) {
    bool hasCharsX = x < CHAR_COLS * 5;
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
    displaySetAddrWindow(x, y1, x, y2);
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
        for (int charRow = 0; charRow < CHAR_ROWS; charRow++) {
            char charIndex = charCol + charRow * CHAR_COLS;
            char character = characters[charIndex];
            blitCharacterVSlice(character, xBit, charRow << 3);
        }
    }

    if (x >= 56 && x <= 72) {
        colourBuffer[target[x - 56]] = X_Yellow;
    }

    displayWritePixels(colourBuffer, y1, y2);
}

/**
 * Get the MPU values, compute FPS, do some vector math to orient the horizon,
 * then render the horizon as a series of columns.
 */
void drawHorizon() {
    digitalWrite(DSO_TEST_PIN, LOW);
    mpuGet();

    printToBuffer(0, 0, mpuAccel.x);
    printToBuffer(1, 0, mpuAccel.y);
    printToBuffer(2, 0, mpuAccel.z);
    printToBuffer(6, 2, mpuTemp);

    uint16_t now = millis();
    float fps = 1000.0f / (now - lastFrame);
    printToBuffer(5, 1, fps);
    lastFrame = now;

    // Transaction
    displayStartWrite();

    // Build a vector to "down", and normalise it, ie 0,0,1
    // given our forward vector is -1,0,0 (depends on sensor orientation),
    // cross product of forward and down should correlate with horizon slope?
    // and angle between forward and down gives horizon offset?
    // dot product of forward and down gives us the cos of the angle between them
    Vector accel = { .x = mpuAccel.x, .y = mpuAccel.y, .z = mpuAccel.z };
    vecNormalise(accel);
    float cosPitch = dotProduct(accel, vecForward);
    Vector vecSide = vecCrossProduct(accel, vecForward);
    float m = vecSide.z / vecSide.y;

    // Find the equation of a line which expresses the horizon.
    // Using slope-intercept y = mx + c. Note that we will need
    // special handling to cope with 90 degrees, since m will be INF
    float c = -70 * cosPitch + 64 - m * 64; // the m*64 is so we don't have to m*(x-64) each loop

    // We write the firstFrame column separately for two reasons.
    // First, we can completely avoid a pair of unnecessary multiplies and additions.
    // Second, if we trigger the DSO after the firstFrame column it makes some measurements a bit easier.
    drawHorizonColumn(0, c, old_c, firstFrame);

    digitalWrite(DSO_TEST_PIN, HIGH);
    // Draw the horizon - brown below the line, blue above.
    for (int16_t x = 1; x < SCREEN_SIZE; x++) {
        drawHorizonColumn(x, m * x + c, old_m * x + old_c, firstFrame);
    }

    old_m = m;
    old_c = c;
    firstFrame = false;

    displayEndWrite();
}

void setup(void) {
    memset(characters, CHAR_SPACE, CHAR_CELLS);
    pinMode(DSO_TEST_PIN, OUTPUT);

    displaySetup();
    mpuSetup();
}

void loop() {
    drawHorizon();
}