/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include <Servo.h>  //Need for Servo pulse output

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Define fire fighting sequence for project 2
enum fireFighting {
  INACTIVE,
  ROTATE,
  ONWARDS,
  OBSTACLE,
  ADJUST_ROBOT,
  ACTIVATE_FAN,
  DEACTIVATE_FAN,
  REVERSE,
  STOP
};

int lengthCounter = 0;
bool startPressed = false;

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 44; // 48
const int ECHO_PIN = 45; // 49
double sonarDistance;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_font_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_font_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;


int speed_val = 2 * 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

///////////////////////////////////////////// Fan
const int fan = 33;


//////////////////////////////////////////// sensing calibration for ir sensors
int rightIR = A15;
double rightIRval;
int leftIR = A14;
double leftIRval;

int rightDiagonalIR = A12;
double rightDiagonalIRval;
int leftDiagonalIR = A13;
double leftDiagonalIRval;


int starterM = 21;

///////////////////////////////////////////// Gyro

const int gyroSensorPin = A3;
int T = 100; // T is the time of one loop, 0.1 sec
int gyroSensorValue = 0; // read out value of sensor
float gyroSupplyVoltage = 5; // supply voltage for gyro
float gyroZeroVoltage = 0; // the value of voltage when gyro is zero
float gyroSensitivity = 0.007; // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1.5; // because of gyro drifting, defining rotation angular velocity less than this value will be ignored
float gyroRate = 0; // read out value of sensor in voltage
float currentAngle = 0; // current angle calculated by angular velocity integral on
byte serialRead = 0; // for serial print control

/////////////////////////////////////////////
// Debug Pins
const int inObstacleState = 4;
const int frontIsMaxLED = 5;

/////////////////////////////////////////

// Phototransistor Pins
int frontPT = A6;
double frontPTdist;

int frontRightPT = A7;
double frontRightPTdist;

int backRightPT = A8;
double backRightPTdist;

int backPT = A10;
double backPTdist;

int backLeftPT = A5;
double backLeftPTdist;

int frontLeftPT = A9;
double frontLeftPTdist;

double idealFrontPTDistance = 15;

int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  //////////////////////////////////////// Phototransistors
  pinMode(frontPT, INPUT);
  frontPTdist = 0;
  pinMode(frontRightPT, INPUT);
  frontRightPTdist = 0;
  pinMode(backRightPT, INPUT);
  backRightPTdist = 0;
  pinMode(backPT, INPUT);
  backPTdist = 0;
  pinMode(backLeftPT, INPUT);
  backLeftPTdist = 0;
  pinMode(frontLeftPT, INPUT);
  frontLeftPTdist = 0;

  /////////////////////////////////////////Gyro

  int i;
  float sum = 0;
  pinMode(gyroSensorPin, INPUT);
  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (i = 0; i < 100; i++) { // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
    gyroSensorValue = analogRead(gyroSensorPin);
    sum += gyroSensorValue;
    delay(5);
  }
  gyroZeroVoltage = sum / 100; // average the sum as the zero drifting


  /////////////////////////////////////////////// debug pins
  pinMode(frontIsMaxLED, OUTPUT);
  digitalWrite(frontIsMaxLED, LOW);

  pinMode(inObstacleState, OUTPUT);

  ///////////////////////////////////////// IR Sensors
  pinMode(rightIR, INPUT);
  rightIRval = 0;

  pinMode(leftIR, INPUT);
  leftIRval = 0;

  pinMode(rightDiagonalIR, INPUT);
  rightDiagonalIRval = 0;

  pinMode(leftDiagonalIR, INPUT);
  leftDiagonalIRval = 0;

  ///////////////////////////////////////// Fan
  pinMode(fan, OUTPUT);
  digitalWrite(fan, LOW);


  /////////////////////////////////////// Sonar Distance
  sonarDistance = 0;

  ///////////////////////////////////////

  pinMode(starterM, INPUT_PULLUP);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial1;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed

}

double LF = 2;
double RF = 2;
double LB = 2;
double RB = 2;
double setValue = 7.6;
double Rw = 27 * pow(10, -3);
double smallL = 43.5 * pow(10, -3);
double bigL = 75.75 * pow(10, -3);
double Vx = 0;
double Vy = 0;
double Wz = 0;
double duration = 0.5;
double sensorOffset = 7.6;
double baseSpeed = 2.5;

bool frontIsMax = false;
bool obstacleDetected = false;

void loop(void) //main loop
{
  //---------------Sensing System Code----------------------------------------

  // Side IR Sensors
  rightIRval = analogRead(rightIR);
  delay(1);
  rightIRval = analogRead(rightIR);
  rightIRval = 3138.9 * pow(rightIRval, -1.045); // y = 3138.9x-1.045

  leftIRval = analogRead(leftIR);
  delay(1);
  leftIRval = analogRead(leftIR);
  leftIRval =  3722.5 * pow(leftIRval, -1.074);

  rightDiagonalIRval = analogRead(rightDiagonalIR);
  delay(1);
  rightDiagonalIRval = analogRead(rightDiagonalIR);
  rightDiagonalIRval = 6704.9 * pow(rightDiagonalIRval, -1.062);

  leftDiagonalIRval = analogRead(leftDiagonalIR);
  delay(1);
  leftDiagonalIRval = analogRead(leftDiagonalIR);
  leftDiagonalIRval = 11202 * pow(leftDiagonalIRval, -1.141); // y = 11202x-1.141

  //Jumper cable to start system without keyboard input.
  if (digitalRead(starterM) == LOW) {
    startPressed = true;
    SerialCom->println("begin");
  }
  ////////////////////////// Phototransistor Values ///////////////////////////
  frontPTdist = analogRead(frontPT);
  delay(1);
  frontPTdist = 762.42 * pow(analogRead(frontPT), -0.652); //y = 762.42x-0.652
  if (frontPTdist > 500) frontPTdist = 500;

  frontRightPTdist = analogRead(frontRightPT);
  delay(1);
  frontRightPTdist = 409.48 * pow(analogRead(frontRightPT), -.497); // y = 498.52x-0.562   //409.48,-.497
  if (frontPTdist < 27) frontRightPTdist = frontRightPTdist + 20;
  if (frontRightPTdist > 500) frontRightPTdist = 500;

  backRightPTdist = analogRead(backRightPT);
  delay(1);
  backRightPTdist = 414.9 * pow(analogRead(backRightPT), -0.545); // y = 414.9x-0.545
  if (backRightPTdist > 500) backRightPTdist = 500;

  backPTdist = analogRead(backPT);
  delay(1);
  backPTdist = 545.46 * pow(analogRead(backPT), -0.57); // y = 545.46x-0.57
  if (backPTdist > 500) backPTdist = 500;

  backLeftPTdist = analogRead(backLeftPT);
  delay(1);
  backLeftPTdist = 498.53 * pow(analogRead(backLeftPT), -0.568); // y = 498.53x-0.568
  if (backLeftPTdist > 500) backLeftPTdist = 500;

  frontLeftPTdist = analogRead(frontLeftPT);
  delay(1);
  frontLeftPTdist = 442.38 * pow(analogRead(frontLeftPT), -0.521); // y = 530.62x-0.58 //442.38, -0.521
  if (frontLeftPTdist > 500) frontLeftPTdist = 500;

  // SerialCom->print("Front: ");
  //SerialCom->print(frontPTdist);
  //  SerialCom->print(", FL: ");
  //  SerialCom->print(frontLeftPTdist);//(frontLeftPTdist);
  //  SerialCom->print(", FR: ");
  //  SerialCom->print(frontRightPTdist); //(frontRightPTdist);
  //  SerialCom->print(", BL: ");
  //  SerialCom->print(backLeftPTdist);
  //  SerialCom->print(" Back: ");
  //  SerialCom->print(backPTdist);
  //  SerialCom->print(", BR: ");
  //  SerialCom->println(backRightPTdist);


  SerialCom->print("RD: ");
  SerialCom->print(rightDiagonalIRval);
  SerialCom->print(", LD: ");
  SerialCom->print(leftDiagonalIRval);
  SerialCom->print(", Right: ");
  SerialCom->print(rightIRval);
  SerialCom->print(", Left: ");
  SerialCom->print(leftIRval);

  SerialCom->print(", Sonar: ");
  SerialCom->println(sonarDistance);


  frontIsMax = ((frontPTdist < frontRightPTdist) && (frontPTdist < backRightPTdist) && (frontPTdist < backPTdist) && (frontPTdist < backLeftPTdist) && (frontPTdist < frontLeftPTdist) && (frontPTdist < 300) && (frontLeftPTdist < 300) && (frontRightPTdist < 300) && !((sonarDistance < 20) && (frontPTdist > 30)));

  obstacleDetected = (((sonarDistance < 12) && (frontPTdist > 30)) || (rightDiagonalIRval < 10) || (leftDiagonalIRval < 10) || (leftIRval < 4) || (rightIRval < 4));

  /////////////////////////////////BASE CODE///////////////////////////////////

  static STATE machine_state = INITIALISING;
  //Finite-state machine Code

  static fireFighting fire_state = INACTIVE;
  //Sequence steps

  digitalWrite(inObstacleState, (fire_state == OBSTACLE));
  digitalWrite(frontIsMaxLED, frontIsMax);

  switch (fire_state) {
    case INACTIVE:
      fire_state = beginOperation();
      break;
    case ROTATE:
      fire_state = rotate();
      break;
    case ONWARDS:
      fire_state =  onwards();
      break;
    case OBSTACLE:
      fire_state =  avoidObstacle();
      break;
    case ADJUST_ROBOT:
      fire_state = adjustRobot();
      break;
    case ACTIVATE_FAN:
      fire_state = activateFan();
      break;
    case DEACTIVATE_FAN:
      fire_state = deactivateFan();
      break;
    case REVERSE:
      fire_state = reverseRobot();
      break;
    case STOP:
      fire_state = stopFireFighting();
      break;
  };

  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop if Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };


} ///////////////////////////////// main loop finish ///////////////////////


//////////////////////////////// Fire Fighting State Functions ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

fireFighting beginOperation() // start controlled operation
{
  if (startPressed == true)
  {
    return ROTATE;
  } else {
    return INACTIVE;
  }
}

fireFighting rotate() {
  // Determine which way to rotate so that max phototransitor value is the front sensor
  if (!frontIsMax) {
    speed_val = 70;
    cw();
    return ROTATE;
  } else {
    speed_val = 150;
    return ONWARDS;
  }
}

double leftGain = 1;
double rightGain = 1;

//Variables for fuzzy logic
double NEAR;
double FAR;
double OKAY;
double SLOW;
double ZERO;
double FAST;
int dirCounter = 0;

fireFighting onwards() // Drive forward until distance
{
  //Generate Membership functions for inputs
  //Membership function for when the robot is NEAR
     if (sonarDistance <= 3) {
      NEAR = 1;
    } else if (sonarDistance >= 30) {
      NEAR = 0;
    } else {
      NEAR = ((-0.05 * sonarDistance) + 1.5);
    }
    //Membership function for when the robot is OKAY
    if (sonarDistance <= 20 && sonarDistance >= 3) {
      OKAY = ((0.1 * sonarDistance) - 1);                          //if sonarDistance = 3 then OKAY = -.7 becomes Zero --> causes system to slow down which is what we want when is close to light

    } else if (sonarDistance <= 30 && sonarDistance >= 20) {
      OKAY = ((-0.1 * sonarDistance) + 3);
    } else {
      OKAY = 0;
    }
    //Membership function for when the robot is FAR
    if (sonarDistance <= 3) {
      FAR = 0;
    } else if (sonarDistance >= 30) {
      FAR = 1;
    } else {
      FAR = ((0.05 * sonarDistance) - 0.5);
    }
    //
    //  //Rules
    SLOW = NEAR; // if sonarDist is NEAR then speed_val is SLOW
    FAST = FAR;  // if sonarDist is FAR then speed_val is FAST
    ZERO = OKAY; // if sonarDist is OKAY then speed_val is ZERO
    //
    //  //Defuzzify speed
    speed_val = (SLOW * 85) + (ZERO * 0) + (FAST * 150); //at some point, is going from a really low value to a negative value which is causing a spike reverse response.




  if (frontIsMax && (leftDiagonalIRval > 18) && (rightDiagonalIRval > 18) && (sonarDistance > 12)) { //if clear path and light is seen by front PT keep going forward
    if (frontLeftPTdist < frontRightPTdist) {
      SerialCom->print("Turning Right: ");
      SerialCom->println((frontRightPTdist - frontLeftPTdist) / frontLeftPTdist / 1.5);
      leftGain = 0.8;
      rightGain = 1 + (frontRightPTdist - frontLeftPTdist) / frontLeftPTdist / 1.5;
      if (rightGain > 3) rightGain = 3;
      controlledForward();
    }
    else if (frontRightPTdist < frontLeftPTdist) {
      SerialCom->print("Turning Left: ");
      SerialCom->println((frontLeftPTdist - frontRightPTdist) / frontRightPTdist / 1.5);
      rightGain = 0.8;
      leftGain = 1 + (frontLeftPTdist - frontRightPTdist) / frontRightPTdist / 1.5;
      if (leftGain > 3) leftGain = 3;
      controlledForward();
    }
    return ONWARDS;
  }
  else if (((frontPTdist < idealFrontPTDistance) && (sonarDistance < 3)) || ((OKAY == 0) && (FAR == 0) && (frontPTdist <= 30))) {
    return ADJUST_ROBOT;
  }
  else if (obstacleDetected) {
    return OBSTACLE;
  }
  else if (((backLeftPTdist < frontPTdist) || (backRightPTdist < frontPTdist)  || (backPTdist < frontPTdist)) && (frontLeftPTdist > 70) && (frontRightPTdist > 70)) {
    cw();
    return ONWARDS;
  }
  else if (frontLeftPTdist < frontRightPTdist) {
    SerialCom->print("Turning Right: ");
    SerialCom->println((frontRightPTdist - frontLeftPTdist) / frontLeftPTdist / 1.5);
    leftGain = 0.8;
    rightGain = 1 + (frontRightPTdist - frontLeftPTdist) / frontLeftPTdist / 1.5;
    if (rightGain > 3) rightGain = 3;
    controlledForward();
    return ONWARDS;
  }
  else if (frontRightPTdist < frontLeftPTdist) {
    SerialCom->print("Turning Left: ");
    SerialCom->println((frontLeftPTdist - frontRightPTdist) / frontRightPTdist / 1.5);
    rightGain = 0.8;
    leftGain = 1 + (frontLeftPTdist - frontRightPTdist) / frontRightPTdist / 1.5;
    if (leftGain > 3) leftGain = 3;
    controlledForward();
    return ONWARDS;
  }
  else
  {
    rightGain = 1;
    leftGain = 1;
    controlledForward(); // change to better controlled motion
    //return ONWARDS;
    return ROTATE;
  }
  //  return ONWARDS;
}

fireFighting avoidObstacle() {
  if (sonarDistance > 12 && leftDiagonalIRval > 14 && rightDiagonalIRval > 14 ) {
    return ONWARDS;
  }
  //Avoid Obstacle
  if ((leftDiagonalIRval < 10) && (rightDiagonalIRval < 10)) {
    return REVERSE;
  }
  else if (leftDiagonalIRval < 10 || leftIRval < 4) {
    speed_val = 150;
    strafe_right();
    return OBSTACLE;
  }
  else if (rightDiagonalIRval < 10 || rightIRval < 4) {
    speed_val = 150;
    strafe_left();
    return OBSTACLE;
  }
  else if (sonarDistance < 12 && frontPTdist > 40) {
    if (rightIRval > leftIRval) {
      speed_val = 150;
      strafe_right();
    }
    else {
      speed_val = 150;
      strafe_left();
    }
    return OBSTACLE;

  }
  else if (!frontIsMax) {
    return ROTATE;
  }
  return ONWARDS;
}


fireFighting adjustRobot() {
  return ACTIVATE_FAN;
}

fireFighting activateFan() {
  if (frontPTdist < idealFrontPTDistance) {
    stop();
    digitalWrite(fan, HIGH);
    return ACTIVATE_FAN;
  } else {
    speed_val = 150;
    return DEACTIVATE_FAN;
  }
}

int counter = 0; //Count the amount of times the fan is activated, two times stops the robot.
fireFighting deactivateFan() {
  digitalWrite(fan, LOW);
  counter = counter + 1;
  if (counter >= 2) {
    counter = 0;
    return STOP;
  } else {
    return REVERSE;
  }
}

fireFighting reverseRobot() {
  reverse();
  delay(300);
  return ROTATE;
}

fireFighting stopFireFighting() {
  stop();
  startPressed = false;
  return INACTIVE;
}

/////////////////////////////////BASE CODE///////////////////////////////////
STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {
  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();

    //#ifndef NO_READ_GYRO
    //    GYRO_reading();
    //#endif

#ifndef NO_HC-SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


    turret_motor.write(pos);

    if (pos == 0)
    {
      pos = 45;
    }
    else
    {
      pos = 0;
    }
  }

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();

    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HC-SR04

// Moving Average Filter Values
const int numReadings = 1;
double readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
double total = 0;                  // the running total
double average = 0;                // the average

void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  //cm = (pulse_width - 145.56)/55.766; // 58.0 , 55.766x + 145.56

  if (pulse_width < 680) {
    cm = 0.0699 * pulse_width - 36.424;
  } else {
    cm = 0.0177 * pulse_width - 1.4283;
  }

  // Moving Average Filter
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = cm;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  //SerialCom->println(average);
  //delay(1);
  sonarDistance = average;

  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->println(average);
    //SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  //SerialCom->print("Analog Range A4:");
  //SerialCom->println(analogRead(A4));
}


void GYRO_reading()
{

  // convert the 0-1023 signal to 0-5v
  gyroRate = (analogRead(gyroSensorPin) * gyroSupplyVoltage) / 1023; // find the voltage
  gyroRate -= (gyroZeroVoltage / 1023 * gyroSupplyVoltage); //(offset by the value of voltage when gyro is zero (still))
  float angularVelocity = gyroRate / gyroSensitivity; // read out voltage divided the gyro sensitivity to calculate the angular velocity, from Data Sheet, gyroSensitivity is 0.007 V/dps

  // if the angular velocity is less than the threshold, ignore it
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    // we are running a loop in T (of T/1000 second).
    float angleChange = angularVelocity / (1000 / T);
    currentAngle += angleChange;
  }
  if (currentAngle < 0) {
    currentAngle += 360; // keep the angle between 0-360
  }
  else if (currentAngle > 359) {
    currentAngle -= 360;
  }

  Serial.print(angularVelocity);
  Serial.print(" ");
  Serial.println(currentAngle);
  // control the time per loop
  delay (T); //will there be issue with using delay here to calculate


  SerialCom->print("Raw GYRO A3:");
  SerialCom->println(analogRead(A3));

  SerialCom->print("Angular Velocity: ");
  SerialCom->print(angularVelocity);

  SerialCom->print("Current Angle ");
  SerialCom->print(currentAngle);

}


//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      //      case 'x':
      //        operationForward ();
      //        SerialCom->println("operationForward");
      //        break;
      case 'g':
      case 'G':
        startPressed = true;
        SerialCom->println("begin");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

//---------------------Motor gains---------------------------
// The following variables are gain values to apply to the motors for each of the motor movements.
// Since there are different movements there are multiple gains for each motor

// Left front motor
int FGLF = 2;         // Forward movement gain left front motor.
int RGLF = 2;         // Reverse movement gain left front motor.
int CCWGLF = 2;       // Counter clockwise movement gain left front motor.
int CWGLF = 2;        // Clockwise movement gain left front motor.
int SLGLF = 2;        // Strafe left movement gain left front motor.
int SRGLF = 2;        // Strafe right movement gain left front motor.

// Right front motor
int FGRF = 2;         // Forward movement gain right front motor.
int RGRF = 2;         // Reverse movement gain right front motor.
int CCWGRF = 2;       // Counter clockwise movement gain right front motor.
int CWGRF = 2;        // Clockwise movement gain right front motor.
int SLGRF = 2;        // Strafe left movement gain right front motor.
int SRGRF = 2;        // Strafe right movement gain right front motor.

// Left rear motor
int FGLR = 2;         // Forward movement gain left rear motor.
int RGLR = 2;         // Reverse movement gain left rear motor.
int CCWGLR = 2;       // Counter clockwise movement gain left rear motor.
int CWGLR = 2;        // Clockwise movement gain left rear motor.
int SLGLR = 2;        // Strafe left movement gain left rear motor.
int SRGLR = 2;        // Strafe right movement gain left rear motor.

// Right rear motor
int FGRR = 2;         // Forward movement gain right rear motor.
int RGRR = 2;         // Reverse movement gain right rear motor.
int CCWGRR = 2;       // Counter clockwise movement gain right rear motor.
int CWGRR = 2;        // Clockwise movement gain right rear motor.
int SLGRR = 2;        // Strafe left movement gain right rear motor.
int SRGRR = 2;        // Strafe right movement gain right rear motor.

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_font_motor.writeMicroseconds(1500);
}

void controlledForward()
{
  left_font_motor.writeMicroseconds(1500 + (leftGain * speed_val));
  left_rear_motor.writeMicroseconds(1500 + (leftGain * speed_val));
  right_rear_motor.writeMicroseconds(1500 - (rightGain * speed_val));
  right_font_motor.writeMicroseconds(1500 - (rightGain * speed_val));
}


void forward()
{
  left_font_motor.writeMicroseconds(1500 + (speed_val));
  left_rear_motor.writeMicroseconds(1500 + (speed_val));
  right_rear_motor.writeMicroseconds(1500 - (speed_val));
  right_font_motor.writeMicroseconds(1500 - (speed_val));
}

void reverse ()
{
  left_font_motor.writeMicroseconds(1500 - (RGLF * speed_val));
  left_rear_motor.writeMicroseconds(1500 - (RGLR * speed_val));
  right_rear_motor.writeMicroseconds(1500 + (RGRR * speed_val));
  right_font_motor.writeMicroseconds(1500 + (RGRF * speed_val));
}

void ccw ()
{
  left_font_motor.writeMicroseconds(1500 - (CCWGLF * speed_val));
  left_rear_motor.writeMicroseconds(1500 - (CCWGLR * speed_val));
  right_rear_motor.writeMicroseconds(1500 - (CCWGRR * speed_val));
  right_font_motor.writeMicroseconds(1500 - (CCWGRF * speed_val));
}

void cw ()
{
  left_font_motor.writeMicroseconds(1500 + (CWGLF * speed_val));
  left_rear_motor.writeMicroseconds(1500 + (CWGLR * speed_val));
  right_rear_motor.writeMicroseconds(1500 + (CWGRR * speed_val));
  right_font_motor.writeMicroseconds(1500 + (CWGRF * speed_val));
}

void strafe_left ()
{
  left_font_motor.writeMicroseconds(1500 - (SLGLF * speed_val));
  left_rear_motor.writeMicroseconds(1500 + (SLGLR * speed_val));
  right_rear_motor.writeMicroseconds(1500 + (SLGRR * speed_val));
  right_font_motor.writeMicroseconds(1500 - (SLGRF * speed_val));
}

void strafe_right ()
{
  left_font_motor.writeMicroseconds(1500 + (SRGLF * speed_val));
  left_rear_motor.writeMicroseconds(1500 - (SRGLR * speed_val));
  right_rear_motor.writeMicroseconds(1500 - (SRGRR * speed_val));
  right_font_motor.writeMicroseconds(1500 + (SRGRF * speed_val));
}




