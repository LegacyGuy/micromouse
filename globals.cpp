#include "globals.h"

const uint8_t rows = 16, cols = 16;

struct cell floodArray[rows * cols]; // 1D array of cell structures

uint8_t targetCells[4], startCell, startDir, currentCell;

uint8_t *values[7] = { &startCell, &(targetCells[0]), &(targetCells[1]), &(targetCells[2]), &(targetCells[3]), &startDir };

int sensorValue[7];

long newPosition1, newPosition2, oldPosition1, oldPosition2;

Encoder myEnc1(2, 8);
Encoder myEnc2(3, 12);
