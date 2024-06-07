#include "globals.h"
#include "floodfill.h"
#include "save_maze.h"
#include <Encoder.h>

// Motor driver pins
const int AIN1 = 5;
const int BIN1 = 6;
const int AIN2 = 4;
const int BIN2 = 7;
const int PWMA = 9;
const int PWMB = 10;
const int STBY = 19;

// Ultrasonic sensor pins
const int trigPinForward = 8;
const int echoPinForward = 9;
const int trigPinLeft = 10;
const int echoPinLeft = 11;
const int trigPinRight = 12;
const int echoPinRight = 13;

// Maze and flood arrays
const int mazeSize = 16;
int maze[mazeSize][mazeSize] = {0};  // 0 = no wall, 1 = wall

// Robot's current position
int posX = 0;
int posY = 0;
int direction = 0; // 0 = forward, 1 = left, 2 = back, 3 = right

// Encoder initialization
Encoder myEnc1(2, 8);
Encoder myEnc2(3, 12);

long distanceTravelled;
int P, D, I, previousError, PIDvalue, error;
int lsp = 100;
int rsp = 100;
float Kp = 1;
float Kd = 10;
float Ki = 0;

int minValues[7], maxValues[7], threshold[7], sensorArray[7];
int wallValues[5];
double encStart1 = 0;
double encStart2 = 0;

// Flood fill and maze variables
const uint8_t rows = 16, cols = 16;
struct cell floodArray[rows * cols];  // Array stores flood value and neighbor data for all cells
uint8_t targetCells[4], startCell, startDir, currentCell;
long newPosition1, newPosition2, oldPosition1, oldPosition2;
int sensorValue[7];

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

void setup() {
  // Set pin modes for motors and sensors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  pinMode(trigPinForward, OUTPUT);
  pinMode(echoPinForward, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  
  pinMode(button, INPUT_PULLUP);
  pinMode(sensor_On_Pin, OUTPUT);

  // Setup ADC for faster analogRead
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);

  // Initialize flood array and maze values
  initializeFloodArray();
  updateMazeValuesFromEEPROM();

  // Initialize encoder positions
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();
  oldPosition1 = newPosition1;
  oldPosition2 = newPosition2;

  // Calibration
  calibrate();
  Serial.begin(9600);
}

void loop() {
  initialise();  // Initialize maze and direction settings
  delay(100);  // Small delay to ensure system stability

  while (currentCell != targetCells[0] && currentCell != targetCells[1] && currentCell != targetCells[2] && currentCell != targetCells[3]) {
    updateWalls();
    flood();
    updateTargetCell();
    goToTargetCell();
    floodArray[currentCell].visited = 1;
  }

  // Save maze state to EEPROM
  updateMazeValuesInEEPROM();
}

// Function to read ultrasonic sensor
int readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  return distance;
}

// Function to update maze array based on sensor readings
void updateMazeArray(int distanceForward, int distanceLeft, int distanceRight) {
  if (distanceForward < 18) {
    // Wall detected in front
    updateMazeCell(posX, posY, direction, 1);
  }
  if (distanceLeft < 18) {
    // Wall detected on the left
    updateMazeCell(posX, posY, (direction + 1) % 4, 1);
  }
  if (distanceRight < 18) {
    // Wall detected on the right
    updateMazeCell(posX, posY, (direction + 3) % 4, 1);
  }
}

// Function to update a specific cell in the maze array
void updateMazeCell(int x, int y, int dir, int value) {
  if (dir == 0) maze[x][y] |= 1;       // Forward
  else if (dir == 1) maze[x][y] |= 2;  // Left
  else if (dir == 2) maze[x][y] |= 4;  // Back
  else if (dir == 3) maze[x][y] |= 8;  // Right
}

// Function to perform flood fill on the flood array
void performFloodFill() {
  for (int i = 0; i < mazeSize; i++) {
    for (int j = 0; j < mazeSize; j++) {
      floodArray[i * mazeSize + j].flood = mazeSize * mazeSize; // Initialize to a high value
    }
  }
  floodArray[posX * mazeSize + posY].flood = 0;

  bool updated = true;
  while (updated) {
    updated = false;
    for (int x = 0; x < mazeSize; x++) {
      for (int y = 0; y < mazeSize; y++) {
        if (floodArray[x * mazeSize + y].flood < mazeSize * mazeSize) {
          int value = floodArray[x * mazeSize + y].flood + 1;
          if ((maze[x][y] & 1) == 0 && floodArray[x * mazeSize + (y + 1)].flood > value) {
            floodArray[x * mazeSize + (y + 1)].flood = value;
            updated = true;
          }
          if ((maze[x][y] & 2) == 0 && floodArray[(x - 1) * mazeSize + y].flood > value) {
            floodArray[(x - 1) * mazeSize + y].flood = value;
            updated = true;
          }
          if ((maze[x][y] & 4) == 0 && floodArray[x * mazeSize + (y - 1)].flood > value) {
            floodArray[x * mazeSize + (y - 1)].flood = value;
            updated = true;
          }
          if ((maze[x][y] & 8) == 0 && floodArray[(x + 1) * mazeSize + y].flood > value) {
            floodArray[(x + 1) * mazeSize + y].flood = value;
            updated = true;
          }
        }
      }
    }
  }
}

// Function to determine the next move
int determineNextMove() {
  int minValue = mazeSize * mazeSize;
  int nextMove = -1;
  if (floodArray[posX * mazeSize + (posY + 1)].flood < minValue && (maze[posX][posY] & 1) == 0) {
    minValue = floodArray[posX * mazeSize + (posY + 1)].flood;
    nextMove = 0; // Forward
  }
  if (floodArray[(posX - 1) * mazeSize + posY].flood < minValue && (maze[posX][posY] & 2) == 0) {
    minValue = floodArray[(posX - 1) * mazeSize + posY].flood;
    nextMove = 1; // Left
  }
  if (floodArray[posX * mazeSize + (posY - 1)].flood < minValue && (maze[posX][posY] & 4) == 0) {
    minValue = floodArray[posX * mazeSize + (posY - 1)].flood;
    nextMove = 2; // Back
  }
  if (floodArray[(posX + 1) * mazeSize + posY].flood < minValue && (maze[posX][posY] & 8) == 0) {
    minValue = floodArray[(posX + 1) * mazeSize + posY].flood;
    nextMove = 3; // Right
  }
  return nextMove;
}

// Function to move the robot to the next cell
void makeMove(int move) {
  if (move == 0) moveForward();
  else if (move == 1) turnLeft();
  else if (move == 2) turnRight();
}

// Function to move the robot forward
void moveForward() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 255);
  delay(1000);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN1, LOW);
  posX += direction == 0 ? 1 : (direction == 2 ? -1 : 0);
  posY += direction == 1 ? 1 : (direction == 3 ? -1 : 0);
}

// Function to turn the robot left
void turnLeft() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 255);
  delay(500);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  direction = (direction + 1) % 4;
}

// Function to turn the robot right
void turnRight() {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 255);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, 255);
  delay(500);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN2, LOW);
  direction = (direction + 3) % 4;
}

// Function to initialize the flood array
void initializeFloodArray() {
  for (int i = 0; i < mazeSize; i++) {
    for (int j = 0; j < mazeSize; j++) {
      floodArray[i * mazeSize + j].flood = mazeSize * mazeSize;
    }
  }
  floodArray[(mazeSize / 2) * mazeSize + (mazeSize / 2)].flood = 0;
}

// Function to update encoder values
void encUpdate() {
  newPosition1 = myEnc1.read();
  newPosition2 = myEnc2.read();

  if (newPosition1 != oldPosition1) {
    oldPosition1 = newPosition1;
  }
  if (newPosition2 != oldPosition2) {
    oldPosition2 = newPosition2;
  }
}

// Function to reset encoder values
void resetEnc() {
  newPosition1 = 0;
  oldPosition1 = -999;
  newPosition2 = 0;
  oldPosition2 = -999;
}

// Function to calibrate sensors
void calibrate() {
  digitalWrite(sensor_On_Pin, HIGH);

  for (int i = 0; i < 5; i++) {
    int j = i;
    if (i > 2) j = i + 3;
    minValues[i] = analogRead(j);
    maxValues[i] = analogRead(j);
  }

  for (int i = 0; i < 10000; i++) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 50);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 50);

    for (int i = 0; i < 5; i++) {
      int j = i;
      if (i > 2) j = i + 3;

      if (analogRead(j) < minValues[i]) {
        minValues[i] = analogRead(j);
      }
      if (analogRead(j) > maxValues[i]) {
        maxValues[i] = analogRead(j);
      }
    }
  }
  digitalWrite(sensor_On_Pin, LOW);
  digitalWrite(AIN1, LOW);
  digitalWrite(BIN2, LOW);
}

// Function to read wall sensor values
void readWall() {
  digitalWrite(sensor_On_Pin, HIGH);
  for (int i = 0; i < 5; i++) {
    int j = i;
    if (i > 2) j = i + 3;
    sensorValue[i] = map(analogRead(j), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
  }
  digitalWrite(sensor_On_Pin, LOW);
}
