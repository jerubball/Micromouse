/**********************************************************\
 *  Micromouse 2018. Mouse Control Program.               * 
 *    Version 1.2.10                                      * 
 *    2018FEB18SUN1809EST                                 * 
 *  New York Institute of Technology.                     * 
 *  Institute of Electrical and Electronics Engineers.    * 
 *    Author: Hasol Im (him@nyit.edu)                     * 
 *    Team:   Nicholas Cannizzo (ncannizz@nyit.edu)       * 
 *            Andre Gustave (agustave@nyit.edu)           * 
 *            Brendan McKenna (bmckenna@nyit.edu)         * 
\**********************************************************/

//----------------------------------------------------------
//  ReadMe
//----------------------------------------------------------

/* 
 *  For use with Arduino/Genuino SAMD/ARM boards only.
 *  Target hardware is Arduino Zero.
 *  Hardware Specification.
 *    3 Sharp GP2Y0A51SK IR analog distance sensors.
 *      Each in Analog In pins 1, 2, 3.
 *    2 DFRobot FIT0441 Brushless DC Motors with Encoder.
 *      Each in Digital pins 4, 5, 6, and 8, 9, 10.
 *    1 Arduino Built-in LED for visual feedback.
 *      Pin 13.
 */

//----------------------------------------------------------
//  Library Specification
//----------------------------------------------------------

#include <StackList.h>
#include <QueueList.h>

//----------------------------------------------------------
//  Variable Declarlation
//----------------------------------------------------------

// constant for pins
const byte ir_left = A3, ir_front = A2, ir_right = A1;
const byte motor_left_pwm = 10, motor_left_dir = 9, motor_left_fg = 8;
const byte motor_right_pwm = 6, motor_right_dir = 5, motor_right_fg = 4;
const byte analog_read_resolution = 12, analog_write_resolution = 10;
const int analog_read_range = 1 << (analog_read_resolution - 1), analog_write_range = 1 << (analog_write_resolution - 1);

// constant for program
byte state;
const byte program_start_init = 1, program_start_wait = 2, program_intermediate = 3, program_stop = 4, program_test = 9;
const byte search_move = 11, search_fix = 12, search_plan = 13;
const byte return_move = 21, return_fix = 22, return_plan = 23;

// constant for locational data
short current_dir, target_dir;
const byte dir_east = 0, dir_north = 2, dir_west = 4, dir_south = 6;
const byte dir_northeast = 1, dir_northwest = 3, dir_southwest = 5, dir_southeast = 7;
const byte dir_limit = 8, dir_mode = 2;
byte current_x, current_y, target_x, target_y;
const byte maze_length = 16;
const byte maze_size = maze_length * 2 - 1;
byte maze[maze_size][maze_size];
const byte encode_size = 8;

// constant for cell data
const byte is_cell = 0, is_horizontal = 1, is_vertical = 2, is_post = 3;
const byte cell_known = 1, cell_blocked = 2, cell_visited = 4;

// search tools
StackList <unsigned short> searchStack;
QueueList <unsigned short> returnQueue;

//----------------------------------------------------------
//  Program Core Definition
//----------------------------------------------------------

/**
 * Arduino-default setup function.
 */
void setup () {
  // set up variables
  state = program_start_init;
  // open serial connection
  Serial.begin (9600);
  // set up analog pin resolutions
  analogReadResolution (analog_read_resolution);
  analogWriteResolution (analog_write_resolution);
  // set up communication pins
  pinMode (LED_BUILTIN, OUTPUT);
  // set up motor pins
  pinMode (motor_left_pwm, OUTPUT);
  pinMode (motor_left_dir, OUTPUT);
  pinMode (motor_left_fg, INPUT);
  pinMode (motor_right_pwm, OUTPUT);
  pinMode (motor_right_dir, OUTPUT);
  pinMode (motor_right_fg, INPUT);
  // set up sensor pins
  pinMode (ir_front, INPUT);
  pinMode (ir_left, INPUT);
  pinMode (ir_right, INPUT);
  // set pin states
  digitalWrite (LED_BUILTIN, LOW);
  setMotor (0, 0);
}

/**
 * Arduino-default loop function.
 */
void loop () {
  doPreLoop ();
  switch (state) {
    case program_start_init:
      state = doProgramStartInit ();
      break;
    case program_start_wait:
      state = doProgramStartWait ();
      break;
    case search_move:
      state = doSearchMove ();
      break;
    case search_fix:
      state = doSearchFix ();
      break;
    case search_plan:
      state = doSearchPlan ();
      break;
    case program_intermediate:
      state = doProgramIntermediate ();
      break;
    case return_move:
      state = doReturnMove ();
      break;
    case return_fix:
      state = doReturnFix ();
      break;
    case return_plan:
      state = doReturnPlan ();
      break;
    case program_stop:
      state = doProgramStop ();
      break;
    case program_test:
      state = doProgramTest ();
      break;
    default:
      Serial.flush ();
      exit (state);
      break;
  }
  doPostLoop ();
}

/**
 * Program-specific pre-loop function.
 */
void doPreLoop () {
  // blinkDelay (LED_BUILTIN, 20, 20, 1);
}

/**
 * Program-specific post-loop function.
 */
void doPostLoop () {
  delay(50);
}

//----------------------------------------------------------
//  Flow Control Definition
//----------------------------------------------------------

byte doProgramStartInit () {
  // initialize variables
  current_x = 0;
  current_y = 0;
  current_dir = 0;
  // initialize maze
  for (byte x = 0; x < maze_size; x++) {
    for (byte y = 0; y < maze_size; y++) {
      maze[x][y] = 0;
    }
  }
  // initialize search problem
  searchStack.push (encodeLocation (current_x, current_y));
  maze[current_x][current_y] |= (cell_known | cell_visited);
  // change state
  return program_start_wait;
}

byte doProgramStartWait () {
  blinkDelay (LED_BUILTIN, 500, 500, 5);
  // change state
  return search_plan;
}

byte doSearchMove () {
  // turn if needed
  if (target_dir != current_dir) {
    short dDir = target_dir - current_dir;
    short sDir = signum (dDir);
    setMotor (-sDir, sDir);
    delay (500);
    setMotor (0, 0);
    current_dir += sDir;
  }
  else {
    int reading = readSensor (ir_front);
    // move until sensor reading.
    if (reading < (analog_read_resolution * 0.4)) {
      setMotor (1.0, 1.0);
    }
    else {
      setMotor (0, 0);
      // if done, increment/decrement current location according to direction
      current_x += getLocationDX (current_dir);
      current_y += getLocationDY (current_dir);
      // mark as visited
      maze[current_x][current_y] |= cell_visited;
      // change state
      return search_fix;
    }
  }
  // keep state
  return search_move;
}

byte doSearchFix () {
  // check for all sensor reading
  int reading = readSensor (ir_front);
  // move until sensor reading.
  if (reading < (analog_read_resolution * 0.5)) {
    setMotor (0.3, 0.3);
  }
  else {
    // if done, increment/decrement current location according to direction
    current_x += getLocationDX (current_dir);
    current_y += getLocationDY (current_dir);
    // mark as visited
    maze[current_x][current_y] |= cell_visited;
    // change state
    return search_plan;
  }
  // keep state
  return search_fix;
}

byte doSearchPlan () {
  if (searchStack.isEmpty ()) {
    // change state
    return program_intermediate;
  }
  else {
    // get location from stack
    unsigned short loc0 = searchStack.peek ();
    byte x0 = decodeLocationX (loc0), y0 = decodeLocationY (loc0);
    // if location is different from current location
    if (x0 != current_x || y0 != current_y) {
      // make target location
      target_x = x0;
      target_y = y0;
      target_dir = getDirection (current_x, current_y, target_x, target_y);
      // change state
      return search_move;
    }
    // get information of reachable cells
    bool neighbor[dir_limit];
    getNeighbors (neighbor, x0, y0);
    short dx0, dy0;
    // check for unknown cells nearby
    bool unknown[dir_limit];
    bool unknown_flag = false;
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      dx0 = getLocationDX (d);
      dy0 = getLocationDY (d);
      unknown[d] = neighbor[d] && ((maze[x0 + dx0][y0 + dy0] & cell_known) == 0);
      unknown_flag = unknown_flag || unknown[d];
    }
    if (unknown_flag) {
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        if (unknown[d]) {
          dx0 = getLocationDX (d);
          dy0 = getLocationDY (d);
          // if there is unknown cell, make it known
          maze[x0 + dx0][y0 + dy0] |= cell_known;
          // check for sensor reading and update information
          // unknown cells should always be aligned with one of the sensors
          if (readSensorDirection (d)) {
            maze[x0 + dx0][y0 + dy0] |= cell_blocked;
          }
        }
      }
    }
    // check for unvisited cells
    bool unvisited[dir_limit];
    bool unvisited_flag = false;
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      dx0 = getLocationDX (d) * 2;
      dy0 = getLocationDY (d) * 2;
      unvisited[d] = neighbor[d] && ((maze[x0 + dx0][y0 + dy0] & cell_visited) == 0);
      unvisited_flag = unvisited_flag || unvisited[d];
    }
    // if some, push and plan.
    if (unvisited_flag) {
      short dir0;
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        short dir0 = d + current_dir;
        if (unvisited[dir0]) {
          dx0 = getLocationDX (d);
          dy0 = getLocationDY (d);
          target_x = x0 + dx0;
          target_y = y0 + dy0;
          target_dir = getDirection (current_x, current_y, target_x, target_y);
          searchStack.push (encodeLocation (target_x, target_y));
          // change state
          return search_move;
        }
      }
    }
    else {
      // if none, pop and continue
      searchStack.pop ();
      // keep state
      return search_plan;
    }
  }
  // this section should be unreachable
  return -1;
}

byte doProgramIntermediate () {
  blinkDelay (LED_BUILTIN, 500, 500, 10);
  return 0;
}

byte doReturnMove () {
  return return_move;
}

byte doReturnFix () {
  return return_fix;
}

byte doReturnPlan () {
  return return_plan;
}

byte doProgramStop () {
  return 0;
}

byte doProgramTest () {
  return program_test;
}

//----------------------------------------------------------
//  Software Utility Definition
//----------------------------------------------------------

/**
 * Apply bitwise AND mask to maze data.
 */
void maskMaze (byte val) {
  for (byte x = 0; x < maze_size; x++) {
    for (byte y = 0; y < maze_size; y++) {
      maze[x][y] &= val;
    }
  }
}

/**
 * Encode two 1-byte coordinates into one 2-byte number.
 */
unsigned short encodeLocation (byte x, byte y) {
  return (x << encode_size) + y;
}

/**
 * Decode 2-byte number's X data.
 */
byte decodeLocationX (unsigned short loc) {
  return loc >> encode_size;
}

/**
 * Decode 2-byte number's Y data.
 */
byte decodeLocationY (unsigned short loc) {
  return loc % (1 << encode_size);
}

/**
 * Determine cell type.
 */
byte getCellType (byte x, byte y) {
  return (x % 2) * 2 + (y % 2);
}

/**
 * Compute delta-X value of given direction.
 */
short getLocationDX (byte d) {
  return short (round (cos (float (d) * PI / dir_limit * 2)));
}

/**
 * Compute delta-Y value of given direction.
 */
short getLocationDY (byte d) {
  return short (round (sin (float (d) * PI / dir_limit * 2)));
}

/**
 * Compute direction between two points.
 */
short getDirection (short x1, short y1, short x2, short y2) {
  return short (round (atan2 (y2 - y1, x2 - x1) / PI * dir_limit / 2 + dir_limit)) % dir_limit;
}

/**
 * Check if given location is a goal post.
 */
bool checkGoalPost (byte x, byte y) {
  return false;
}

/**
 * Check if given location is a goal cell.
 */
bool checkGoalCell (byte x, byte y) {
  return false;
}

void getNeighbors (bool neighbors[], byte x, byte y) {
  neighbors[dir_east] = (x < maze_size - 2);
  neighbors[dir_north] = (y < maze_size - 2);
  neighbors[dir_west] = (x > 1);
  neighbors[dir_south] = (y > 1);
}

short signum (float val) {
  if (val > 0) {
    return 1;
  }
  else if (val < 0) {
    return -1;
  }
  else {
    return 0;
  }
}

//----------------------------------------------------------
//  Hardware Utility Definition
//----------------------------------------------------------

/**
 * Set motor power. Arguments are in range [-1.0, 1.0].
 */
void setMotor (float left, float right) {
  bool left_dir = (left > 0), right_dir = (right > 0);
  int left_power = int (round (abs (left) * analog_write_range)), right_power = int (round (abs (right) * analog_write_range));
  digitalWrite (motor_left_dir, left_dir);
  digitalWrite (motor_right_dir, right_dir);
  analogWrite (motor_left_pwm, left_power);
  analogWrite (motor_right_pwm, right_power);
}

/**
 * Read sensor in given pin.
 */
int readSensor (byte id) {
  int reading = analogRead (id);
  return analog_read_range - reading;
}

/**
 * Determine if given direction is blocked.
 */
bool readSensorDirection (byte dir) {
  short offset = dir - current_dir;
  offset -= short (round (float (offset) / dir_limit) * dir_limit);
  offset /= dir_mode;
  int reading = readSensor (byte (ir_front + offset));
  return reading > (analog_read_resolution * 0.5);
}

/**
 * Do High-Low blink on given pin with on and off duration for repeated amount.
 */
void blinkDelay (byte id, short on, short off, short rep) {
  for (short i = 0; i < rep; i++) {
    digitalWrite (id, HIGH);
    delay (on);
    digitalWrite (id, LOW);
    delay (off);
  }
}

//----------------------------------------------------------
//  Milestones
//----------------------------------------------------------

/* 
 *  Phase 1: Implement Basic Robot Control
 *  Phase 2: Implement Search Stage
 *  Phase 3: Implement Return Plan Stage
 *  Phase 4: Implement Encoder Motor Control
 *  Phase 5: Implement Diagonal Robot Control
 */

//----------------------------------------------------------
//  Changelog
//----------------------------------------------------------

/* 
 *  1.2:  2018FEB18SUN
 *        All search stage is implemented.
 *        Micromouse may traverse any arbitrary size maze, and return to starting point.
 *  
 *  1.1:  2018FEB13TUE
 *        Utility functions are implemented.
 *        setMotor and readSensor functions are implemented.
 *        AnalogRead resolution set to 12 bits. AnalogWrite resolution set to 10 bits.
 *  
 *  1.0:  2018FEB08THU
 *        Structure of program is implemented.
 *        switch statements and functions for search stage and plan stage added.
 */

