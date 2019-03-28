/**********************************************************\
 *  Micromouse 2018. Mouse Control Program.               * 
 *    Version 1.6.21                                      * 
 *    2018MAR20TUE1320EDT                                 * 
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
 *      Respective wire colors are: Yellow, Blue, Green.
 *    1 Arduino Built-in LED for visual feedback.
 *      Defaults to Digital pin 13.
 *    2 Push-to-open switches. Red and Blue.
 *      Each in Digital pins 1 and 2.
 *  Instruction Overview.
 *     1. Make sure two buttons are connected and not pressed.
 *        Long-delay short-blink (200 ms on, 800 ms off) indicates attention.
 *     2. Press and hold Blue button to discard previous maze data.
 *        Short-delay short-blink (100 ms on, 400 ms off) will appear 6 times (3 seconds).
 *        If this button is mistaken, press hardware reset switch.
 *        No blink indicates that program will discard data by default.
 *     3. Press and hold Red button to start the program.
 *        Long-delay equal-blink (500 ms on, 500 ms off) will appear 3 times (3 seconds).
 *        The action will be confirmed by Short-delay equal-blink (250 ms on, 250 ms off) indicator for 6 times (3 seconds).
 *     4. During the execution of program, press and hold Blue button to issue save command.
 *        Short-delay short-blink (100 ms on, 400 ms off) will appear 6 times (3 seconds).
 *        NOTE: Issue this command only if new data needs to be stored. Write cycle of Flash memory is limited.
 *        The action will be confirmed by Short-delay equal-blink (250 ms on, 250 ms off) indicator for 6 times (3 seconds).
 *        If this button is mistaken, the command can be canceled during confirmation blink by pressing hardware reset switch.
 *        It may be possible to quickly switch to Red button for regular termination.
 *     5. During the execution of program, press and hold Red button to issue stop command.
 *        Long-delay equal-blink (500 ms on, 500 ms off) will appear 3 times (3 seconds).
 *        The action will be confirmed by Short-delay equal-blink (250 ms on, 250 ms off) indicator for 6 times (3 seconds).
 *        All unsaved data will be discarded.
 */

// Include this directive for use with other program.
/*
#define setup micromouseSetup
#define loop micromouseLoop
#include "C:\...\sketch_micromouse_program\sketch_micromouse_program.ino"
#undef setup
#undef loop
*/


//------------------------------------------------------------------------------
//  Library Specification
//------------------------------------------------------------------------------

#include <StackList.h>
#include <QueueList.h>
#define EEPROM_EMULATION_SIZE 4096
#include <FlashAsEEPROM.h>


//------------------------------------------------------------------------------
//  Variable Declaration
//------------------------------------------------------------------------------

// constant for pins
const byte button_red = 1, button_blue = 2, buzzer = A0;
const byte ir_left = A3, ir_front = A4, ir_right = A5;
const byte motor_left_pwm = 8, motor_left_dir = 9, motor_left_fg = 10;
const byte motor_right_pwm = 4, motor_right_dir = 5, motor_right_fg = 6;
const byte analog_read_resolution = 12, analog_write_resolution = 10;
const long analog_read_range = (1 << analog_read_resolution) - 1, analog_write_range = (1 << analog_write_resolution) - 1;

// constant for motor control
const bool reverse_motor_left = false, reverse_motor_right = true;
const byte move_forward = 0, move_clockwise = 1, move_counterclockwise = 2, move_backward = 3;
const byte shift_left = 4, shift_right = 8;
const float one_inch = 73, one_inch_factor = 0, one_inch_offset = 5; // 20000 100000 70000
const float one_rotation = 1190, one_rotation_factor = 100, one_rotation_offset = 20; // 110000 180000 800000 700000
const long encoder_read_duration = 100000, encoder_read_interval = 10000; // 500000 ; 
const float power_high = 0.30, power_medium = 0.15, power_low = 0.09;
const float power_diff = 0.005; // 0.002 0.005
const float ir_range_over = 0.80, ir_range_center = 0.60; // 0.89 0.88 ; 0.44 0.60
volatile long motor_encoder_left, motor_encoder_right;

// constant for program
byte current_state, previous_state;
const byte program_start_init = 1, program_start_wait = 2, program_intermediate = 3, program_pause = 4, program_stop = 5;
const byte movement_turn = 11, movement_move = 12, movement_fix = 13;
const byte search_init = 25, search_plan = 26; // search_turn = 21, search_move = 22, search_fix = 23;
const byte solve_init = 35, solve_plan = 36; // solve_turn = 31, solve_move = 32, solve_fix = 33;
const byte return_init = 45, return_plan = 46; // return_turn = 41, return_move = 42, return_fix = 43;
const byte program_test = 51;
volatile unsigned long current_time, previous_time;

// constant for locational data
short current_dir, target_dir;
const byte dir_east = 0, dir_north = 2, dir_west = 4, dir_south = 6;
const byte dir_northeast = 1, dir_northwest = 3, dir_southwest = 5, dir_southeast = 7;
const byte dir_limit = 8, dir_mode = 2;
byte current_x, current_y, target_x, target_y, start_x, start_y, goal_x, goal_y;
const byte encode_size = 8;
const byte maze_length = 6; // modify this number for different sized maze, Default: 16
const byte maze_size = maze_length * 2 - 1;
byte maze[maze_size][maze_size];

// constant for cell data
const byte is_cell = 0, is_horizontal = 1, is_vertical = 2, is_post = 3;
const byte cell_known = 1, cell_blocked = 2, cell_visited = 4, cell_planned = 8;
//  00 00 0 0 0 0
//  do di p v b k

// search tools
StackList <unsigned short> searchStack;
QueueList <unsigned short> searchQueue;
QueueList <unsigned short> resultQueue;

// test tools
bool turn;


//------------------------------------------------------------------------------
//  Function Name Declaration
//------------------------------------------------------------------------------

// Program Core
void setup ();  // line 150
void loop ();  // line 200
void doEscapeLoop ();  // line 250
void doPreLoop ();  // line 300
void doPostLoop ();  // line 300

// Flow Control Definition
byte doProgramStartInit ();  // line 300
byte doProgramStartWait ();  // line 300
byte doProgramIntermediate ();  // line 350
byte doProgramPause ();
byte doProgramStop ();  // line 
byte doMovementTurn ();  // line 350
byte doMovementMove ();  // line 400
byte doMovementFix ();  // line 400
byte doSearchInit ();  // line 
byte doSearchPlan ();  // line 450
byte doSolveInit ();  //
byte doSolvePlan ();  // line 550
byte doReturnInit ();
byte doReturnPlan ();  // line 550
byte doProgramTest ();  // line 

// Software Utility Definition
void readMaze ();  // line 500
void writeMaze ();  // line 600
void maskMaze (byte val);  // line 600
unsigned short encodeLocation (byte x, byte y);  // line 650
byte decodeLocationX (unsigned short loc);  // line 650
byte decodeLocationY (unsigned short loc);  // line 650
byte getCellType (byte x, byte y);  // line 650
short getLocationDX (byte d);  // line 650
short getLocationDY (byte d);  // line 650
short getDirection (short x1, short y1, short x2, short y2);  // line 650
bool checkGoalPost (byte x, byte y);  // line 650
bool checkGoalCell (byte x, byte y);  // line 700
unsigned short findGoalPost ();  // line
void getNeighbors (bool neighbors[], byte x, byte y);  // line 700
short signum (float val);  // line 700
short signumOld (float val);  // line 700

// Hardware Utility Definition
void zeroMotor (bool mode, short dur, byte rep);  // line 750
void setMotor (float left, float right);  // line 750
void setMotorEncoder (long left, long right);  // line 750
void readMotorEncoderOld ();  // line 800
void interruptMotorEncoderLeft ();  // line 800
void interruptMotorEncoderRight ();  // line 800
bool runMotorEncoder (float power, byte mode, float dist);  // line 800
bool shiftMotorEncoder (float power, byte mode, float dist);  // line 
float readSensor (byte id);  // line 900
bool readSensorDirection (byte dir);  // line 900
bool confirmButton (byte id, bool mode, byte bid, short on, short off, byte rep); // line 950
void blinkDelay (byte id, short on, short off, byte rep);  // line 950
void ringBuzzer (byte id, float freq, short dur, float power);  // line 
float getFreq (short note);  // line 
void playWeAreTheChampion ();  // line



//------------------------------------------------------------------------------
//  Program Core Definition
//------------------------------------------------------------------------------

/**
 * Arduino-default setup function.
 */
void setup () {
  // set up variables
  current_time = micros ();
  current_state = program_start_init;
  turn = false;
  // open serial connection
  Serial.begin (9600);
  // set up analog pin resolutions
  analogReadResolution (analog_read_resolution);
  analogWriteResolution (analog_write_resolution);
  // set up communication pins
  pinMode (button_red, INPUT_PULLUP);
  pinMode (button_blue, INPUT_PULLUP);
  pinMode (LED_BUILTIN, OUTPUT);
  pinMode (buzzer, OUTPUT);
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
  // set up interrupts
  attachInterrupt (digitalPinToInterrupt (motor_left_fg), interruptMotorEncoderLeft, CHANGE);
  attachInterrupt (digitalPinToInterrupt (motor_right_fg), interruptMotorEncoderRight, CHANGE);
  // set pin states
  digitalWrite (LED_BUILTIN, LOW);
  zeroMotor (HIGH, 10, 3);
  //setMotor (0, 0);
  //setMotorEncoder (0, 0);
}

/**
 * Arduino-default loop function.
 */
void loop () {
  doEscapeLoop ();
  //doPreLoop ();
  switch (current_state) {
    case program_start_init:
      current_state = doProgramStartInit ();
      break;
    case program_start_wait:
      current_state = doProgramStartWait ();
      break;
    case program_intermediate:
      current_state = doProgramIntermediate ();
      break;
    case program_pause:
      current_state = doProgramPause ();
      break;
    case program_stop:
      current_state = doProgramStop ();
      break;
    case movement_turn:
      current_state = doMovementTurn ();
      break;
    case movement_move:
      current_state = doMovementMove ();
      break;
    case movement_fix:
      current_state = doMovementFix ();
      break;
    case search_init:
      current_state = doSearchInit ();
      break;
    case search_plan:
      current_state = doSearchPlan ();
      break;
    case solve_init:
      current_state = doSolveInit ();
      break;
    case solve_plan:
      current_state = doSolvePlan ();
      break;
    case return_init:
      current_state = doReturnInit ();
      break;
    case return_plan:
      current_state = doReturnPlan ();
      break;
    case program_test:
      current_state = doProgramTest ();
      break;
    default:
      Serial.println ("Program Terminated with Exit State: " + String (current_state));
      Serial.flush ();
      exit (current_state);
      break;
  }
  doPostLoop ();
}

/**
 * Program-specific escape check function.
 */
void doEscapeLoop () {
  // if any button is pressed
  if ((current_state > 2) && (digitalRead (button_red) == HIGH || digitalRead (button_blue) == HIGH)) {
    // stop movement
    setMotor (0, 0);
    // write button
    if (confirmButton (button_blue, HIGH, LED_BUILTIN, 100, 400, 6)) { // short-delay short-blink
      blinkDelay (LED_BUILTIN, 250, 250, 6); // short-delay equal-blink
      writeMaze ();
      // NOTE: only enable this line for final program. write cycle of Flash ROM is limited.
      EEPROM.commit ();
      current_state = program_stop;
      blinkDelay (LED_BUILTIN, 250, 250, 6); // short-delay equal-blink
    }
    // stop button
    if (confirmButton (button_red, HIGH, LED_BUILTIN, 500, 500, 3)) { // long-delay equal-blink
      blinkDelay (LED_BUILTIN, 250, 250, 6); // short-delay equal-blink
      current_state = program_stop;
      blinkDelay (LED_BUILTIN, 250, 250, 6); // short-delay equal-blink
    }
  }
}

/**
 * Program-specific pre-loop function.
 */
void doPreLoop () {
  
}

/**
 * Program-specific post-loop function.
 */
void doPostLoop () {
  delay(10);
}


//------------------------------------------------------------------------------
//  Flow Control Definition
//------------------------------------------------------------------------------

byte doProgramStartInit () {
  //previous_time = current_time;
  //current_time = micros ();
  // initialize variables
  start_x = 0;
  start_y = 0;
  current_x = start_x;
  current_y = start_y;
  current_dir = 0;
  // reset encoder
  // setMotor (0, 0);
  zeroMotor (HIGH, 10, 3);
  setMotorEncoder (0, 0);
  // initialize maze
  for (byte x = 0; x < maze_size; x++) {
    for (byte y = 0; y < maze_size; y++) {
      maze[x][y] = 0;
    }
  }
  // change state
  previous_state = program_start_init;
  return program_start_wait;
  //return program_test;
}

byte doProgramStartWait () {
  // ensure both buttons are ready
  while (digitalRead (button_red) == HIGH || digitalRead (button_blue) == HIGH) {
    blinkDelay (LED_BUILTIN, 200, 800, 1); // long-delay short-blink
  }
  bool readFromEEPROM = EEPROM.isValid ();
  // wait for start signal
  while (!confirmButton (button_red, HIGH, LED_BUILTIN, 500, 500, 3)) { // long-delay equal-blink
    // listen for no EEPROM read signal
    if (readFromEEPROM && confirmButton (button_blue, HIGH, LED_BUILTIN, 100, 400, 6)) { // short-delay short-blink
      readFromEEPROM = false;
    }
  }
  blinkDelay (LED_BUILTIN, 250, 250, 6); // short-delay equal-blink
  previous_state = program_start_wait;
  // read from EEPROM
  if (readFromEEPROM) {
    readMaze ();
    Serial.println ("Solve Problem Selected.");
    // change state
    return solve_init;
  }
  else {
    Serial.println ("Search Problem Selected.");
    // change state
    return search_init;
  }
  // this section should be unreachable
  return -1;
}

byte doProgramIntermediate () {
  playWeAreTheChampion ();
  return return_init;
}

byte doProgramPause () {
  setMotor (0, 0);
  return program_pause;
}

byte doProgramStop () {
  
  return 0;
}

byte doMovementTurn () {
  if (target_dir == current_dir) {
    // change state
    return movement_move;
  }
  else {
    short dir0 = target_dir - current_dir;
    if (abs(dir0) > dir_limit / 2) {
      dir0 -= signum (dir0) * dir_limit;
    }
    if (runMotorEncoder (power_medium, (dir0 > 0) ? move_counterclockwise : move_clockwise, abs (float (dir0)) / dir_limit)) {
      // reset encoder
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
      // update direction
      current_dir += dir0;
      if (current_dir < 0) {
        current_dir += dir_limit;
      }
      // change state
      return movement_move;
    }
    // keep state
    return movement_turn;
  }
}

byte doMovementMove () {
  if (runMotorEncoder (power_medium, move_forward, 7.61)) {  // 7.5833 7.59 7.595 7.6
    // reset encoder
    zeroMotor (HIGH, 10, 3);
    setMotorEncoder (0, 0);
    // if done, increment/decrement current location according to direction
    current_x += getLocationDX (current_dir);
    current_y += getLocationDY (current_dir);
    // mark as visited
    maze[current_x][current_y] |= cell_visited;
    // change state
    return movement_fix;
  }
  // keep state
  return movement_move;
}

byte doMovementFix () {
  const float sensor_threshold_front = 0.01, sensor_front_offset = 0.075;
  const float sensor_threshold_side = 0.08, shift_dist = 0.6;
  float sensor_front = readSensor (ir_front);
  float sensor_left = readSensor (ir_left), sensor_right = readSensor (ir_right);
  bool check1 = true, check2 = false;
  float sensor_level;
  
  // adjust rotation
  if (sensor_left > ir_range_over && sensor_right > ir_range_over) {
    // no nearby walls, do not adjust rotation
    check1 = true;
  }
  else if (sensor_left > ir_range_over) {
    // no left wall
    sensor_level = ir_range_center - sensor_right;
    if (sensor_level > sensor_threshold_side) {
      while (!shiftMotorEncoder (power_medium, shift_left, shift_dist));
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
    }
    else if (sensor_level < (- sensor_threshold_side)) {
      while (!shiftMotorEncoder (power_medium, shift_right, shift_dist));
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
    }
    else {
      check1 = true;
    }
  }
  else if (sensor_right > ir_range_over) {
    // no right wall
    sensor_level = ir_range_center - sensor_left;
    if (sensor_level > sensor_threshold_side) {
      while (!shiftMotorEncoder (power_medium, shift_right, shift_dist));
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
    }
    else if (sensor_level < (- sensor_threshold_side)) {
      while (!shiftMotorEncoder (power_medium, shift_left, shift_dist));
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
    }
    else {
      check1 = true;
    }
  }
  else {
    sensor_level = sensor_right - sensor_left;
    // both walls
    if (sensor_level > sensor_threshold_side * 2) {
      while (!shiftMotorEncoder (power_medium, shift_right, shift_dist));
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
    }
    else if (sensor_level < (- sensor_threshold_side * 2)) {
      while (!shiftMotorEncoder (power_medium, shift_left, shift_dist));
      zeroMotor (HIGH, 10, 3);
      setMotorEncoder (0, 0);
    }
    else {
      check1 = true;
    }
  }
  
  // adjust position
  if (check1) {
    if (sensor_front > ir_range_over) {
      // no front wall
      check2 = true;
    }
    else {
      // front wall
      sensor_level = ir_range_center + sensor_front_offset - sensor_front;
      if (sensor_level > sensor_threshold_front) {
        setMotor ((- power_low), (- power_low));
      }
      else if (sensor_level < (- sensor_threshold_front)) {
        setMotor (power_low, power_low);
      }
      else {
        check2 = true;
      }
    }
  }
  // finished to adjust
  if (check2) {
    setMotor (0, 0);
    // reset encoder
    zeroMotor (HIGH, 10, 3);
    setMotorEncoder (0, 0);
    // if done, increment/decrement current location according to direction
    current_x += getLocationDX (current_dir);
    current_y += getLocationDY (current_dir);
    // mark as visited
    maze[current_x][current_y] |= cell_visited;
    // change state
    return previous_state;
  }
  // keep state
  return movement_fix;
}

byte doSearchInit () {
  // initialize search problem
  searchStack = StackList <unsigned short> ();
  searchStack.push (encodeLocation (current_x, current_y));
  maze[current_x][current_y] |= (cell_known | cell_visited);
  previous_state = search_init;
  // change state
  return search_plan;
}

byte doSearchPlan () {
  
  /*while (!Serial.available ());
  while (Serial.available ()) {
    Serial.flush ();
    //Serial.read ();
  }
  */
  
  previous_state = search_plan;
  zeroMotor (HIGH, 10, 3);
  if (searchStack.isEmpty ()) {
    Serial.println ("Search Problem Finished.");
    // change state
    return program_pause;
  }
  else {
    // get location from stack
    unsigned short loc0 = searchStack.peek ();
    byte x0 = decodeLocationX (loc0), y0 = decodeLocationY (loc0);
    Serial.println ("Search @ (" + String(x0) + ", " + String(y0) + ")");
    // if location is different from current location
    if (x0 != current_x || y0 != current_y) {
      Serial.println ("Now @ (" + String(current_x) + ", " + String(current_y) + ")");
      Serial.println ("Move to location.");
      // make target location
      target_x = x0;
      target_y = y0;
      target_dir = getDirection (current_x, current_y, target_x, target_y);
      // change state
      return movement_turn;
    }
    // get information of reachable cells
    bool neighbor[dir_limit];
    getNeighbors (neighbor, x0, y0);
    Serial.println ("Neighbor State: ");
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      Serial.print (String (neighbor[d]) + " ");
    }
    Serial.println ();
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
    Serial.println ("Unknown State: ");
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      Serial.print (String (unknown[d]) + " ");
    }
    Serial.println ();
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
            Serial.println (String(d) + " is blocked.");
            maze[x0 + dx0][y0 + dy0] |= cell_blocked;
          }
          else {
            Serial.println (String(d) + " is not blocked.");
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
    Serial.println ("Unvisited State: ");
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      Serial.print (String (unvisited[d]) + " ");
    }
    Serial.println ();
    // if some, push and plan
    if (unvisited_flag) {
      short dir0;
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        dir0 = (d + current_dir) % dir_limit;
        if (unvisited[dir0]) {
          dx0 = getLocationDX (dir0);
          dy0 = getLocationDY (dir0);
          // add adjacent cell to target
          target_x = x0 + dx0 * 2;
          target_y = y0 + dy0 * 2;
          target_dir = getDirection (current_x, current_y, target_x, target_y);
          Serial.println ("Next @ (" + String(target_x) + ", " + String(target_y) + "), in " + String(dir0));
          searchStack.push (encodeLocation (target_x, target_y));
          // change state
          return movement_turn;
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

byte doSolveInit () {
  // initialize solve problem
  maskMaze (0b00000011);
  searchQueue = QueueList <unsigned short> ();
  resultQueue = QueueList <unsigned short> ();
  searchQueue.push (encodeLocation (current_x, current_y));
  maze[current_x][current_y] |= (cell_known | cell_planned);
  // setup necessary variables
  unsigned short loc0;
  byte x0, y0;
  short dx0, dy0;
  bool neighbor[dir_limit];
  bool unplanned[dir_limit];
  bool unplanned_flag;
  // perform search
  bool found = false;
  while (!found && !searchQueue.isEmpty ()) {
    loc0 = searchQueue.peek ();
    x0 = decodeLocationX (loc0);
    y0 = decodeLocationY (loc0);
    Serial.println ("Search @ (" + String(x0) + ", " + String(y0) + ")");
    // if goal, stop
    if (checkGoalCell (x0, y0)) {
      Serial.println ("Goal found.");
      goal_x = x0;
      goal_y = y0;
      found = true;
    }
    else {
      Serial.println ("Is not a goal.");
      // get information of reachable cells
      getNeighbors (neighbor, x0, y0);
      // check for unplanned cells
      unplanned_flag = false;
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        dx0 = getLocationDX (d);
        dy0 = getLocationDY (d);
        // check if path is known
        unplanned[d] = neighbor[d] && ((maze[x0 + dx0][y0 + dy0] & cell_known) != 0);
        // then check if path is not blocked
        unplanned[d] = unplanned[d] && ((maze[x0 + dx0][y0 + dy0] & cell_blocked) == 0);
        // then check if cell is not planned
        unplanned[d] = unplanned[d] && ((maze[x0 + dx0 * 2][y0 + dy0 * 2] & cell_planned) == 0);
        unplanned_flag = unplanned_flag || unplanned[d];
      }
      Serial.println ("Unplanned State: ");
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        Serial.print (String (unplanned[d]) + " ");
      }
      Serial.println ();
      // if some, push and plan
      if (unplanned_flag) {
        short dir1, dir0 = (maze[x0][y0] >> 4) % (1 << 2) * dir_mode;
        byte x1, y1;
        for (byte d = 0; d < dir_limit; d += dir_mode) {
          dir1 = (d + dir0) % dir_limit;
          if (unplanned[dir1]) {
            dx0 = getLocationDX (dir1);
            dy0 = getLocationDY (dir1);
            // get loation
            x1 = x0 + dx0 * 2;
            y1 = y0 + dy0 * 2;
            // mark as planned
            maze[x1][y1] |= cell_planned;
            // indicate input direction
            //maze[x1][y1] &= ((1 << 4) - 1);
            maze[x1][y1] |= ((dir1 / dir_mode) << 4);
            searchQueue.push (encodeLocation (x1, y1));
          }
        }
      }
      // pop current location.
      searchQueue.pop ();
    }
  }
  // solution is found
  if (found) {
    byte d0, d1;
    byte x1, y1;
    Serial.println ("Path from goal is: ");
    while (x0 != current_x || y0 != current_y) {
      // get input direction
      d0 = (maze[x0][y0] >> 4) % (1 << 2);
      d1 = (d0 * 2 + (dir_limit / 2)) % dir_limit;
      Serial.println ("(" + String(x0) + ", " + String(y0) + ") d0: " + String(d0) + ", d1:" + String(d1));
      // get location
      dx0 = getLocationDX (d1);
      dy0 = getLocationDY (d1);
      x1 = x0 + dx0 * 2;
      y1 = y0 + dy0 * 2;
      // indicate output direction;
      maze[x1][y1] &= ((1 << 6) - 1);
      maze[x1][y1] |= (d0 << 6);
      x0 = x1;
      y0 = y1;
    }
  }
  /*while (!Serial.available ());
  while (Serial.available ()) {
    Serial.flush ();
    Serial.read ();
  }*/  
  previous_state = solve_init;
  // change state
  return solve_plan;
}

byte doSolvePlan () {
  previous_state = solve_plan;
  if (current_x == goal_x && current_y == goal_y) {
    // change state
    return program_intermediate;
  }
  else {
    target_dir = (maze[current_x][current_y] >> 6) * dir_mode;
    short dx0 = getLocationDX (target_dir), dy0 = getLocationDY (target_dir);
    target_x = current_x + dx0 * 2;
    target_y = current_y + dy0 * 2;
    // change state
    return movement_turn;
  }
  // this section should be unreachable
  return -1;
}

byte doReturnInit () {
  // initialize return problem
  maskMaze (0b00000011);
  searchQueue = QueueList <unsigned short> ();
  resultQueue = QueueList <unsigned short> ();
  searchQueue.push (encodeLocation (current_x, current_y));
  maze[current_x][current_y] |= (cell_known | cell_planned);
  // setup necessary variables
  unsigned short loc0;
  byte x0, y0;
  short dx0, dy0;
  bool neighbor[dir_limit];
  bool unplanned[dir_limit];
  bool unplanned_flag;
  // perform search
  bool found = false;
  while (!found && !searchQueue.isEmpty ()) {
    loc0 = searchQueue.peek ();
    x0 = decodeLocationX (loc0);
    y0 = decodeLocationY (loc0);
    Serial.println ("Search @ (" + String(x0) + ", " + String(y0) + ")");
    // if goal, stop
    if (x0 == start_x && y0 == start_y) {
      Serial.println ("Start found.");
      found = true;
    }
    else {
      Serial.println ("Is not start.");
      // get information of reachable cells
      getNeighbors (neighbor, x0, y0);
      // check for unplanned cells
      unplanned_flag = false;
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        dx0 = getLocationDX (d);
        dy0 = getLocationDY (d);
        // check if path is known
        unplanned[d] = neighbor[d] && ((maze[x0 + dx0][y0 + dy0] & cell_known) != 0);
        // then check if path is not blocked
        unplanned[d] = unplanned[d] && ((maze[x0 + dx0][y0 + dy0] & cell_blocked) == 0);
        // then check if cell is not planned
        unplanned[d] = unplanned[d] && ((maze[x0 + dx0 * 2][y0 + dy0 * 2] & cell_planned) == 0);
        unplanned_flag = unplanned_flag || unplanned[d];
      }
      Serial.println ("Unplanned State: ");
      for (byte d = 0; d < dir_limit; d += dir_mode) {
        Serial.print (String (unplanned[d]) + " ");
      }
      Serial.println ();
      // if some, push and plan
      if (unplanned_flag) {
        short dir1, dir0 = (maze[x0][y0] >> 4) % (1 << 2) * dir_mode;
        byte x1, y1;
        for (byte d = 0; d < dir_limit; d += dir_mode) {
          dir1 = (d + dir0) % dir_limit;
          if (unplanned[dir1]) {
            dx0 = getLocationDX (dir1);
            dy0 = getLocationDY (dir1);
            // get loation
            x1 = x0 + dx0 * 2;
            y1 = y0 + dy0 * 2;
            // mark as planned
            maze[x1][y1] |= cell_planned;
            // indicate input direction
            //maze[x1][y1] &= ((1 << 4) - 1);
            maze[x1][y1] |= ((dir1 / dir_mode) << 4);
            searchQueue.push (encodeLocation (x1, y1));
          }
        }
      }
      // pop current location.
      searchQueue.pop ();
    }
  }
  // solution is found
  if (found) {
    byte d0, d1;
    byte x1, y1;
    Serial.println ("Path from start is: ");
    while (x0 != current_x || y0 != current_y) {
      // get input direction
      d0 = (maze[x0][y0] >> 4) % (1 << 2);
      d1 = (d0 * 2 + (dir_limit / 2)) % dir_limit;
      Serial.println ("(" + String(x0) + ", " + String(y0) + ") d0: " + String(d0) + ", d1:" + String(d1));
      // get location
      dx0 = getLocationDX (d1);
      dy0 = getLocationDY (d1);
      x1 = x0 + dx0 * 2;
      y1 = y0 + dy0 * 2;
      // indicate output direction;
      maze[x1][y1] &= ((1 << 6) - 1);
      maze[x1][y1] |= (d0 << 6);
      x0 = x1;
      y0 = y1;
    }
  }
  previous_state = return_init;
  // change state
  return return_plan;
}

byte doReturnPlan () {
  previous_state = return_plan;
  if (current_x == start_x && current_y == start_y) {
    // change state
    return program_pause;
  }
  else {
    target_dir = (maze[current_x][current_y] >> 6) * dir_mode;
    short dx0 = getLocationDX (target_dir), dy0 = getLocationDY (target_dir);
    target_x = current_x + dx0 * 2;
    target_y = current_y + dy0 * 2;
    // change state
    return movement_turn;
  }
  // this section should be unreachable
  return -1;
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
/*
byte mazesave[11][11] = {
  {0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001},
  {0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000001},
  {0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001},
  {0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000001},
  {0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001},
  {0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001},
  {0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001},
  {0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000001},
  {0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001},
  {0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000011, 0b00000001, 0b00000001, 0b00000001, 0b00000001},
  {0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001, 0b00000001}};
*/
/**
 * Read maze data from virtual EEPROM
 */
void readMaze () {
  for (byte x = 0; x < maze_size; x++) {
    for (byte y = 0; y < maze_size; y++) {
      //maze[x][y] = mazesave[x][y];
      maze[x][y] = EEPROM.read(x * maze_size + y);
    }
  }
}

/**
 * Write maze data to virtual EEPROM
 */
void writeMaze () {
  for (byte x = 0; x < maze_size; x++) {
    for (byte y = 0; y < maze_size; y++) {
      EEPROM.write(x * maze_size + y, maze[x][y]);
    }
  }
}

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
    // get information of reachable cells
    bool neighbor[dir_limit];
    getNeighbors (neighbor, x, y);
    for (byte d = 0; d < dir_limit; d += dir_mode) {
      dx = getLocationDX (d);
      dy = getLocationDY (d);
      isGoal = isGoal && neighbor[d] && ((maze[x + dx][y + dy] & cell_blocked) == 0);
    }
    return isGoal;
  }
}

/**
 * Check if given location is a goal cell.
 */
bool checkGoalCell (byte x, byte y) {
  bool isGoal = false;
  short dx, dy;
  // get information of reachable cells
  bool neighbor[dir_limit];
  getNeighbors (neighbor, x, y);
  for (byte d = 1; d < dir_limit; d += dir_mode) {
    dx = getLocationDX (d);
    dy = getLocationDY (d);
    isGoal = isGoal || (neighbor[d] && checkGoalPost (x + dx, y + dy));
  }
  return isGoal;
}

/**
 * Find goal post.
 */
unsigned short findGoalPost () {
  for (byte x = 1; x < maze_size; x += 2) {
    for (byte y = 1; y < maze_size; y += 2) {
      if (checkGoalPost (x, y)) {
        return encodeLocation (x, y);
      }
    }
  }
  return 0;
}

/**
 * Compute truth values for existence of neighbor cells.
 */
void getNeighbors (bool neighbors[], byte x, byte y) {
  neighbors[dir_east] = (x < maze_size - 2);
  neighbors[dir_northeast] = (y < maze_size - 2) && (x < maze_size - 2);
  neighbors[dir_north] = (y < maze_size - 2);
  neighbors[dir_northwest] = (y < maze_size - 2) && (x > 1);
  neighbors[dir_west] = (x > 1);
  neighbors[dir_southwest] = (y > 1) && (x > 1);
  neighbors[dir_south] = (y > 1);
  neighbors[dir_southeast] = (y > 1) && (x < maze_size - 2);
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

void zeroMotor (bool mode, short dur, byte rep) {
  digitalWrite (motor_left_pwm, HIGH);
  digitalWrite (motor_right_pwm, HIGH);
  for (byte i = 0; i < rep; i++) {
    digitalWrite (motor_left_dir, mode);
    digitalWrite (motor_right_dir, mode);
    mode = !mode;
    delay (dur);
  }
}

/**
 * Set motor power. Arguments are in range [-1.0, 1.0].
 */
void setMotor (float left, float right) {
  bool left_dir = (left < 0) ^ reverse_motor_left, right_dir = (right < 0) ^ reverse_motor_right;
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
 * Update encoder value based on time.
 * This function is deprecated.
 */
void readMotorEncoderOld () {
  // update time
  previous_time = current_time;
  current_time = micros ();
  unsigned long time_diff = current_time - previous_time;
  double encoder_read_ratio = double (time_diff) / encoder_read_interval;
  // change encoder value
  unsigned long left_value = pulseIn (motor_left_fg, HIGH, encoder_read_duration);
  unsigned long right_value = pulseIn (motor_right_fg, HIGH, encoder_read_duration);
  motor_encoder_left += (unsigned long) round (left_value * encoder_read_ratio);
  motor_encoder_right += (unsigned long) round (right_value * encoder_read_ratio);
}

/**
 * Interrupt service routine for left motor.
 */
void interruptMotorEncoderLeft () {
  if (!digitalRead (motor_left_dir) ^ reverse_motor_left) {
    motor_encoder_left++;
  }
  else {
    motor_encoder_left--;
  }
}

/**
 * Interrupt service routine for right motor.
 */
void interruptMotorEncoderRight () {
  if (!digitalRead (motor_right_dir) ^ reverse_motor_right) {
    motor_encoder_right++;
  }
  else {
    motor_encoder_right--;
  }
}

/**
 * Runs motor with encoder control. Will try to adjust for average value.
 */
bool runMotorEncoder (float power, byte mode, float dist) {
  short left_sign = ((mode & 2) == 0) ? 1 : -1, right_sign = ((mode & 1) == 0) ? 1 : -1;
  float limit;
  if (left_sign == right_sign) {
    limit = dist * (one_inch - one_inch_factor * power) - one_inch_offset;
  }
  else {
    limit = dist * (one_rotation - one_rotation_factor * power) - one_rotation_offset;
  }
  bool state = (abs(motor_encoder_left) + abs(motor_encoder_right)) >= (limit * 2);
  float sensor_left = readSensor (ir_left), sensor_right = readSensor (ir_right);
  if (state) {
    // job done
    setMotor (0, 0);
    return true;
  }
  else if (left_sign != right_sign) {
    // any turns
    setMotor (power * left_sign, power * right_sign);
  }
  else {
    float power_adjust;
    if (sensor_left > ir_range_over && sensor_right > ir_range_over) {
      // no nearby walls
      setMotor (power * left_sign, power * right_sign);
    }
    else if (sensor_left > ir_range_over) {
      // no left wall
      power_adjust = power * (ir_range_center - sensor_right);
      setMotor ((power - power_adjust) * left_sign, (power + power_adjust) * right_sign);
    }
    else if (sensor_right > ir_range_over) {
      // no right wall
      power_adjust = power * (ir_range_center - sensor_left);
      setMotor ((power + power_adjust) * left_sign, (power - power_adjust) * right_sign);
    }
    else {
      // use both walls to guide
      power_adjust = power * (sensor_right - sensor_left);
      setMotor ((power + power_adjust) * left_sign, (power - power_adjust) * right_sign);
    }
  }
  // continue job
  return false;
}

/**
 * Shifts motor with encoder control. Four stages should come in one action.
 */
bool shiftMotorEncoder (float power, byte mode, float dist) {
  float limit = dist * (one_inch - one_inch_factor * power) - one_inch_offset;
  if (mode == shift_left) {
    // glide left
    if (motor_encoder_right < 0) {
      if (motor_encoder_left < 0) {
        // job done
        setMotor (0, 0);
        return true;
      }
      else {
        // stage 4: move left motor backwards
        setMotor ((- power), 0);
      }
    }
    else if (motor_encoder_left > limit) {
      // stage 3: move right motor backwards
      setMotor (0, (- power));
    }
    else if (motor_encoder_right > limit) {
      // stage 2: move left motor forwards
      setMotor (power, 0);
    }
    else {
      // stage 1: move right motor forwards
      setMotor (0, power);
    }
  }
  else if (mode == shift_right) {
    // glide right
    if (motor_encoder_left < 0) {
      if (motor_encoder_right < 0) {
        // job done
        setMotor (0, 0);
        return true;
      }
      else {
        // stage 4: move right motor backwards
        setMotor (0, (- power));
      }
    }
    else if (motor_encoder_right > limit) {
      // stage 3: move left motor backwards
      setMotor ((- power), 0);
    }
    else if (motor_encoder_left > limit) {
      // stage 2: move right motor forwards
      setMotor (0, power);
    }
    else {
      // stage 1: move left motor forwards
      setMotor (power, 0);
    }
  }
  else {
    // invalid mode
    return true;
  }
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
  float reading = readSensor (byte (ir_front - offset));
  return reading < ir_range_over;
}

/**
 * Checks if given pin is in selected mode for given duration with visual feedback.
 */
bool confirmButton (byte id, bool mode, byte bid, short on, short off, byte rep) {
  if (digitalRead (id) != mode) {
    return false;
  }
  else {
    for (byte i = 0; i < rep; i++) {
      digitalWrite (bid, HIGH);
      delay (on);
      if (digitalRead (id) != mode) {
        digitalWrite (bid, LOW);
        return false;
      }
      digitalWrite (bid, LOW);
      delay (off);
      if (digitalRead (id) != mode) {
        return false;
      }
    }
    return true;
  }
}

/**
 * Do High-Low blink on given pin with on and off duration for repeated amount.
 */
void blinkDelay (byte id, short on, short off, byte rep) {
  for (short i = 0; i < rep; i++) {
    digitalWrite (id, HIGH);
    delay (on);
    digitalWrite (id, LOW);
    delay (off);
  }
}

/**
 * Ring buzzer with given frequency and duration.
 */
void ringBuzzer (byte id, float freq, short dur, float power) {
  if (freq > 0) {
    long value = round (500000 / freq);
    long cycles = round (freq * dur / 1000);
    long strength = power * analog_write_range;
    for (long i=0; i < cycles; i++) {
      analogWrite (id, strength);
      delayMicroseconds (value);
      analogWrite (id, 0);
      delayMicroseconds(value);
    }
  }
  else {
    delay (dur);
  }
}

/**
 * Gets frequency of given note, 0 being A0 (440 Hz).
 */
float getFreq (short note) {
  return 440 * pow (2, note / 12.0);
}


//------------------------------------------------------------------------------
//  Sound Utility Definition
//------------------------------------------------------------------------------

void playWeAreTheChampion () {
  float power = 0.6;
  short beat = 300;
  short offset = 2;
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 4, power);
  ringBuzzer (buzzer, getFreq(7 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(7 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(3 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(0 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(0 + offset), beat * 4, power);
  ringBuzzer (buzzer, 0, beat * 5, power);
  ringBuzzer (buzzer, getFreq(3 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 4, power);
  ringBuzzer (buzzer, getFreq(10 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(12 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(15 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(12 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(7 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 4, power);
  ringBuzzer (buzzer, 0, beat * 2, power);
  ringBuzzer (buzzer, 0, beat * 4, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(3 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(5 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(3 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(1 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(13 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(12 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(13 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(12 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(10 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(12 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(13 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(12 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(13 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(11 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 2, power);
  ringBuzzer (buzzer, getFreq(13 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(11 + offset), beat * 3, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 3, power);
  ringBuzzer (buzzer, 0, beat * 4, power);
  ringBuzzer (buzzer, getFreq(6 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(3 + offset), beat * 1, power);
  ringBuzzer (buzzer, getFreq(8 + offset), beat * 6, power);
}



//------------------------------------------------------------------------------
//  Milestones
//------------------------------------------------------------------------------

/* 
 *  Phase 1: Implement Basic Robot Control
 *  Phase 2: Implement Search Stage
 *  Phase 3: Implement Encoder Motor Control
 *  Phase 4: Implement Solve and Return Stage
 *  Phase 5: Implement Continuous Robot Control
 *  Phase 6: Implement Diagonal Robot Control
 */


//------------------------------------------------------------------------------
//  Changelog
//------------------------------------------------------------------------------

/* 
 *  1.6:  2018MAR20TUE
 *        Solve and return methods fully implemented.
 *  
 *  1.5:  2018MAR16FRI
 *        Encoder tracking is implemented as interrupt service routines.
 *        Movement stages are now independent from plan stages.
 *        It is confirmed that Micromouse can traverse any maze.
 *  
 *  1.4:  2018MAR13TUE
 *        Save and Load of maze data using EEPROM Emulation added.
 *  
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

 
