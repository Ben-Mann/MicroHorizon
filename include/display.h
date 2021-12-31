#ifndef MICROHORIZON_DISPLAY_H
#define MICROHORIZON_DISPLAY_H

#include <SPI.h>

#define SCREEN_SIZE  128
#define LAST_SCREEN_INDEX (SCREEN_SIZE - 1)

void displaySetup();
void displaySetAddrWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void displayWritePixels(uint16_t *colors, uint16_t y1, uint16_t y2);
void displayStartWrite();
void displayEndWrite();

#endif //MICROHORIZON_DISPLAY_H
