#ifndef globals_h
#define globals_h

#include <stdint.h>
#include <Encoder.h>

// Direction constants
#define north 0
#define east 1
#define south 2
#define west 3

// Sensor positions
#define leftSensor 0
#define diagonalLeftSensor 1
#define centreSensor 2
#define diagonalRightSensor 3
#define rightSensor 4

// Threshold for detecting walls
#define wallThreshold 120

// Pin definitions
#define sensor_On_Pin 17
#define button 9

// Utility macros
#define absolute(number) (((number) > 0)? (number) : (-(number)))
#define minimum(num1, num2) (((num1) < (num2))? (num1) : (num2))

// Cell structure to store flood value, wall neighbors, and visit status
struct cell {
  uint8_t flood;
  uint8_t neighbours;
  uint8_t visited;
};

// External variable declarations
extern struct cell floodArray[]; // 1D array of cell structures

extern const uint8_t rows, cols;

extern uint8_t startCell, startDir, targetCells[], currentCell;

extern uint8_t *values[];

extern int sensorValue[];

extern long newPosition1, newPosition2, oldPosition1, oldPosition2;

extern Encoder myEnc1;
extern Encoder myEnc2;

#endif
