/**********************************************************\
 *  Micromouse 2018. Mouse Control Program.               * 
 *    Version 1.3.15                                      * 
 *    2018MAR08THU1500EST                                 * 
 *  New York Institute of Technology.                     * 
 *  Institute of Electrical and Electronics Engineers.    * 
 *    Author: Hasol Im (him@nyit.edu)                     * 
 *    Team:   Nicholas Cannizzo (ncannizz@nyit.edu)       * 
 *            Andre Gustave (agustave@nyit.edu)           * 
 *            Brendan McKenna (bmckenna@nyit.edu)         * 
\**********************************************************/


//------------------------------------------------------------------------------
//  ReadMe
//------------------------------------------------------------------------------

/* 
 *  For use with Arduino/Genuino SAMD/ARM boards only.
 *  Target hardware is Arduino Zero.
 *  Hardware Specification.
 *    3 Sharp GP2Y0A51SK IR analog distance sensors.
 *      Each in Analog In pins 1, 2, 3.
 *    2 DFRobot FIT0441 Brushless DC Motors with Encoder.
 *      Each in Digital pins 4, 5, 6, and 8, 9, 10.
 *      Respective wire colors are: Green, Blue, Yellow.
 *    1 Arduino Built-in LED for visual feedback.
 *      Pin 13.
 */

// Include this directive for use with other program.
/*
#define setup setup_micromouse
#define loop loop_micromouse
#include "C:\...\sketch_micromouse_program\sketch_micromouse_program.ino"
#undef setup
#undef loop
*/


//------------------------------------------------------------------------------
//  Library Specification
//------------------------------------------------------------------------------

#include <StackList.h>
#include <QueueList.h>


//------------------------------------------------------------------------------
//  Variable Declarlation
//------------------------------------------------------------------------------

// constant for pins
const byte ir_left = A3, ir_front = A2, ir_right = A1;
const byte motor_left_pwm = 10, motor_left_dir = 9, motor_left_fg = 8;
const byte motor_right_pwm = 6, motor_right_dir = 5, motor_right_fg = 4;
const byte analog_read_resolution = 12, analog_write_resolution = 10;
const int analog_read_range = (1 << analog_read_resolution) - 1, analog_write_range = (1 << analog_write_resolution) - 1;

// constant for motor control
const bool reverse_motor_left = true, reverse_motor_right = false;
const byte move_forward = 0, move_clockwise = 1, move_counterclockwise = 2, move_backward = 3;
const float one_inch = 20000, one_rotation = 110000;
const int encoder_read_duration = 500000;
long motor_encoder_left, motor_encoder_right;

// constant for program
byte state;
const byte program_start_init = 1, program_start_wait = 2, program_intermediate = 3, program_stop = 4, program_test = 9;
const byte search_turn = 11, search_move = 12, search_fix = 13, search_plan = 14;
const byte return_turn = 21, return_move = 22, return_fix = 23, return_plan = 24;

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

// test tools
bool turn;


//------------------------------------------------------------------------------
//  Function Name Definition
//------------------------------------------------------------------------------

// Program Core
void setup ();  // line 150
void loop ();  // line 150
void doPreLoop ();  // line 200
void doPostLoop ();  // line 200

// Flow Control Definition
byte doProgramStartInit ();  // line 250
byte doProgramStartWait ();  // line 250
byte doSearchTurn ();  // line 250
byte doSearchMove ();  // line 300
byte doSearchFix ();  // line 300
byte doSearchPlan ();  // line 300
byte doProgramIntermediate ();  // line 400
byte doReturnTurn ();  // line 400
byte doReturnMove ();  // line 400
byte doReturnFix ();  // line 450
byte doReturnPlan ();  // line 450
byte doProgramStop ();  // line 450
byte doProgramTest ();  // line 450

// Software Utility Definition
void maskMaze (byte val);  // line 
unsigned short encodeLocation (byte x, byte y);  // line 450
byte decodeLocationX (unsigned short loc);  // line 500
byte decodeLocationY (unsigned short loc);  // line 500
byte getCellType (byte x, byte y);  // line 500
short getLocationDX (byte d);  // line 500
short getLocationDY (byte d);  // line 500
short getDirection (short x1, short y1, short x2, short y2);  // line 500
bool checkGoalPost (byte x, byte y);  // line 550
bool checkGoalCell (byte x, byte y);  // line 550
void getNeighbors (bool neighbors[], byte x, byte y);  // line 550
short signum (float val);  // line 550
short signumOld (float val);  // line 600

// Hardware Utility Definition
void setMotor (float left, float right);  // line 600
void setMotorEncoder (long left, long right);  // line 600
bool runMotorEncoder (float power, byte mode, float dist);  // line 600
bool runMotorEncoderOld (float power, byte mode, float dist);  // line 650
float readSensor (byte id);  // line 700
bool readSensorDirection (byte dir);  // line 700
void blinkDelay (byte id, short on, short off, short rep);  // line 700




//------------------------------------------------------------------------------
//  Program Core Definition
//------------------------------------------------------------------------------

/**
 * Arduino-default setup function.
 */
void setup () {
  // set up variables
  state = program_start_init;
  turn = false;
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
  setMotorEncoder (0, 0);
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
    case search_turn:
      state = doSearchTurn ();
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
    case return_turn:
      state = doReturnTurn ();
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
  // change encoder value.
  motor_encoder_left += pulseIn (motor_left_fg, HIGH, encoder_read_duration);
  motor_encoder_right += pulseIn (motor_right_fg, HIGH, encoder_read_duration);
  // blinkDelay (LED_BUILTIN, 20, 20, 1);
}

/**
 * Program-specific post-loop function.
 */
void doPostLoop () {
  delay(50);
}


//------------------------------------------------------------------------------
//  Flow Control Definition
//------------------------------------------------------------------------------

byte doProgramStartInit () {
  // initialize variables
  current_x = 0;
  current_y = 0;
  current_dir = 0;
  // reset encoder
  setMotorEncoder (0, 0);
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
  //return program_start_wait;
  return program_test;
}

byte doProgramStartWait () {
  blinkDelay (LED_BUILTIN, 500, 500, 5);
  // change state
  return search_plan;
}

byte doSearchTurn () {
  if (target_dir != current_dir) {
    // change state
    return search_move;
  }
  else {
    short dir0 = target_dir - current_dir;
    // TODO: FILL
    if (false) {//runMotorEncoder ()) {
      current_dir += dir0;
      // reset encoder
      setMotorEncoder (0, 0);
      // change state
      return search_move;
    }
    // keep state
    return search_turn;
  }
}

byte doSearchMove () {
  
  /*
  float reading = readSensor (ir_front);
  // move until sensor reading
  if (reading < 0.4) {
    setMotor (1.0, 1.0);
  }
  else {
  */
  setMotor (1.0, 1.0);
  delay (500);
  setMotor (0, 0);
  // if done, increment/decrement current location according to direction
  current_x += getLocationDX (current_dir);
  current_y += getLocationDY (current_dir);
  // TODO: FILL
  if (false) {//runMotorEncoder ()) {
    // mark as visited
    maze[current_x][current_y] |= cell_visited;
    // reset encoder
    setMotorEncoder (0, 0);
    // change state
    return search_fix;
  }
  // keep state
  return search_move;
}

byte doSearchFix () {
  // check for all sensor reading
  float reading = readSensor (ir_front);
  // move until sensor reading
  if (reading < 0.5) {
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
          // if there is unknown path and cell, make it known
          maze[x0 + dx0][y0 + dy0] |= cell_known;
          maze[x0 + dx0 * 2][y0 + dy0 * 2] |= cell_known;
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
      dx0 = getLocationDX (d);
      dy0 = getLocationDY (d);
      // check if path is not blocked
      unvisited[d] = neighbor[d] && ((maze[x0 + dx0][y0 + dy0] & cell_blocked) == 0);
      // then check if cell is not visited
      unvisited[d] = unvisited[d] && ((maze[x0 + dx0 * 2][y0 + dy0 * 2] & cell_visited) == 0);
      unvisited_flag = unvisited_flag || unvisited[d];
    }
    // if some, push and plan
    if (unvisited_flag) {
      short dir0;
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        dir0 = d + current_dir;
        if (unvisited[dir0]) {
          dx0 = getLocationDX (d);
          dy0 = getLocationDY (d);
          // add adjacent cell to target
          target_x = x0 + dx0 * 2;
          target_y = y0 + dy0 * 2;
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

byte doReturnTurn () {
  return return_turn;
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
  float sensor_left = readSensor(ir_left), sensor_right = readSensor (ir_right), sensor_front = readSensor (ir_front);
  Serial.println ("Left: " + String (sensor_left) + "  Right: " + String (sensor_right) + "  Front: " + String (sensor_front));
  Serial.println ("Left: " + String (motor_encoder_left) + "  Right: " + String (motor_encoder_right));
  if (turn) {
    if (runMotorEncoder (0.14, move_counterclockwise, 0.25)) {
      turn = false;
      setMotorEncoder (0, 0);
      // blinkDelay (LED_BUILTIN, 250, 250, 6);
      // return 0;
    }
  }
  else {
    if (sensor_front < 0.67 || runMotorEncoder (0.14, move_forward, 38)) {
      setMotor (0, 0);
      turn = true;
      setMotorEncoder (0, 0);
    }
  }
  return program_test;
}




//------------------------------------------------------------------------------
//  Software Utility Definition
//------------------------------------------------------------------------------

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
  if (getCellType (x, y) != is_post) {
    return false;
  }
  else {
    // all posts are safe with neighbor array index
    bool isGoal = true;
    short dx, dy;
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      dx = getLocationDX (d);
      dy = getLocationDY (d);
      isGoal = isGoal && ((maze[x + dx][y + dy] & cell_blocked) == 0);
    }
    return false;
  }
}

/**
 * Check if given location is a goal cell.
 */
bool checkGoalCell (byte x, byte y) {
  bool isGoal = false;
  short dx, dy;
  for (byte d = 1; d < dir_limit; d += dir_mode) {
    dx = getLocationDX (d);
    dy = getLocationDY (d);
    isGoal = isGoal || checkGoalCell (x + dx, y + dy);
  }
  return isGoal;
}

/**
 * Compute truth values for existance of neighbor cells.
 */
void getNeighbors (bool neighbors[], byte x, byte y) {
  neighbors[dir_east] = (x < maze_size - 2);
  neighbors[dir_north] = (y < maze_size - 2);
  neighbors[dir_west] = (x > 1);
  neighbors[dir_south] = (y > 1);
}

short signum (float val) {
  return (val > 0) ? 1 : (val < 0 ? -1 : 0);
}

short signumOld (float val) {
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


//------------------------------------------------------------------------------
//  Hardware Utility Definition
//------------------------------------------------------------------------------

/**
 * Set motor power. Arguments are in range [-1.0, 1.0].
 */
void setMotor (float left, float right) {
  bool left_dir = (left > 0) ^ reverse_motor_left, right_dir = (right > 0) ^ reverse_motor_right;
  int left_power = analog_write_range - int (round (abs (left) * analog_write_range));
  int right_power = analog_write_range - int (round (abs (right) * analog_write_range));
  digitalWrite (motor_left_dir, left_dir);
  digitalWrite (motor_right_dir, right_dir);
  analogWrite (motor_left_pwm, left_power);
  analogWrite (motor_right_pwm, right_power);
}

/**
 * Sets encoder value to specified input.
 */
void setMotorEncoder (long left, long right) {
  motor_encoder_left = left;
  motor_encoder_right = right;
}

/**
 * Runs motor with encoder control. Will try to adjust for average value.
 */
bool runMotorEncoder (float power, byte mode, float dist) {
  const float diff = 0.002;
  short left_sign = ((mode & 2) == 0) ? 1 : -1, right_sign = ((mode & 1) == 0) ? 1 : -1;
  float limit;
  if (left_sign == right_sign) {
    limit = dist * one_inch;
  }
  else {
    limit = dist * one_rotation;
  }
  bool state = (abs(motor_encoder_left) + abs(motor_encoder_right)) >= (limit * 2);
  float sensor_left = readSensor (ir_left), sensor_right = readSensor (ir_right);
  if (state) {
    // job done.
    setMotor (0, 0);
    return true;
  }
  else {
    if (sensor_left > 0.89 && sensor_right > 0.89) {
      setMotor (power * left_sign, power * right_sign);
    }
    else if (sensor_left > 0.89) {
      if (sensor_right < 0.44) {
        setMotor ((power - diff) * left_sign, (power + diff) * right_sign);
      }
      else {
        setMotor ((power + diff) * left_sign, (power - diff) * right_sign); 
      }
    }
    else if (sensor_right > 0.89) {
      if (sensor_left < 0.44) {
        setMotor ((power + diff) * left_sign, (power - diff) * right_sign);
      }
      else {
        setMotor ((power - diff) * left_sign, (power + diff) * right_sign);
      }
    }
    else {
      if (sensor_left > sensor_right) {
        setMotor ((power - diff) * left_sign, (power + diff) * right_sign);
      }
      else if (sensor_left < sensor_right) {
        setMotor ((power + diff) * left_sign, (power - diff) * right_sign);
      }
      else {
        setMotor (power * left_sign, power * right_sign);
      }
    }
  }
  // continue job
  return false;
}

/**
 * Sets encoder value to specified input. Will try to adjust for exact value.
 */
bool runMotorEncoderOld (float power, byte mode, float dist) {
  const float slow_factor = 1.0;
  short left_sign = ((mode & 2) == 0) ? 1 : -1, right_sign = ((mode & 1) == 0) ? 1 : -1;
  float limit;
  if (left_sign == right_sign) {
    limit = dist * one_inch;
  }
  else {
    limit = dist * one_rotation;
  }
  bool left_state = abs(motor_encoder_left) >= limit, right_state = abs(motor_encoder_right) >= limit;
  if (left_state && right_state) {
    // job done
    setMotor (0, 0);
    return true;
  }
  else if (left_state || right_state) {
    if (left_state) {
      // adjust only right motor
      setMotor (0, power * slow_factor * right_sign);
    }
    if (right_state) {
      // adjust only left motor
      setMotor (power * slow_factor * left_sign, 0);
    }
  }
  else {
    setMotor (power * left_sign, power * right_sign);
  }
  // change encoder value.
  // motor_encoder_left += pulseIn (motor_left_fg, HIGH, encoder_read_duration);
  // motor_encoder_right += pulseIn (motor_right_fg, HIGH, encoder_read_duration);
  // continue job
  return false;
}

/**
 * Read sensor in given pin.
 */
float readSensor (byte id) {
  int reading = analogRead (id);
  return float (analog_read_range - reading) / analog_read_range;
}

/**
 * Determine if given direction is blocked.
 */
bool readSensorDirection (byte dir) {
  short offset = dir - current_dir;
  offset -= short (round (float (offset) / dir_limit) * dir_limit);
  offset /= dir_mode;
  float reading = readSensor (byte (ir_front + offset));
  return reading < 0.895;
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




//------------------------------------------------------------------------------
//  Milestones
//------------------------------------------------------------------------------

/* 
 *  Phase 1: Implement Basic Robot Control
 *  Phase 2: Implement Search Stage
 *  Phase 3: Implement Return Plan Stage
 *  Phase 4: Implement Encoder Motor Control
 *  Phase 5: Implement Diagonal Robot Control
 */


//------------------------------------------------------------------------------
//  Changelog
//------------------------------------------------------------------------------

/* 
 *  1.3:  2018MAR06TUE
 *        runMotorEncoder functions are implemented.
 *        Motor controls now work properly.
 *        This file now can be included for other files.
 *  
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

 
