//Definitions:

#define leftMotors_IN1 2
#define leftMotors_IN2 10
#define leftMotors_IN3 12
#define leftMotors_IN4 13

#define rightMotors_IN1 A8
#define rightMotors_IN2 7
#define rightMotors_IN3 8
#define rightMotors_IN4 9

#define leftMotors_ENA 5
#define leftMotors_ENB 6
#define rightMotors_ENA 3
#define rightMotors_ENB 4

#define spikeSpeed 254
#define spikeDelay 100

#define leftMost_sensor A0
#define left_sensor A1
#define center_sensor A2
#define right_sensor A3
#define rightMost_sensor A4

//--------------------------------------------------------------------------------------------------------------------------------------------------------

//Variables:

//Motor Speeds
int forwardSpeeds[4] = {90, 90, 120, 160};
//int turnLeftSpeeds[4] = {-140, -140, 140, 140};
int turnLeftSpeeds[4] = {-160, -160, 160, 160};
int turnRightSpeeds[4] = {110, 110, -150, -150};
int turnAroundSpeeds[4] = {-150, -150, 180, 180};
int maxSpeed = 254;

// IR Sensor readings
int sensors[5];

// PID Proportionality Parameters
float Kp = 100;
float Ki = 0;
float Kd = 100;

//Extra PID variables
float currPosition = 0;
float error = 0;
float prevError = 0;
float derivative = 0;
float integral = 0;
int PIDvalue = 0;

//For storing path details
char path[100] = {};
int pathLength = 0;

int finishDetections = 0;

//For ensuring correctness when updating the path.
unsigned long prevTime=0;
unsigned long junctionDetectionTime=0;

//Indicate whether we should take left/right turns or not.
bool leftFlag = false;
bool rightFlag = false;

//If true, the robot will stop, since it has reached the end.
bool doneFlag = false;

//Indicates whether to solve the maze by taking left turns first or right turns first.
bool moveLeftFirst = true;

//Indicates that the robot is in roundTwo.
bool firstTrialFlag = true;
int pathIndex = 0;

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotors_IN1, OUTPUT);
  pinMode(leftMotors_IN2, OUTPUT);
  pinMode(leftMotors_IN3, OUTPUT);
  pinMode(leftMotors_IN4, OUTPUT);

  pinMode(rightMotors_IN1, OUTPUT);
  pinMode(rightMotors_IN2, OUTPUT);
  pinMode(rightMotors_IN3, OUTPUT);
  pinMode(rightMotors_IN4, OUTPUT);

  pinMode(leftMotors_ENA, OUTPUT);
  pinMode(leftMotors_ENB, OUTPUT);
  
  pinMode(rightMotors_ENA, OUTPUT);
  pinMode(rightMotors_ENB, OUTPUT);

  Serial.begin(9600);

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {

  // put your main code here, to run repeatedly:
  //set_MotorSpeeds(254, 254, -254, -254);
  //delay(100);
  
  readSensors();
  if (checkFinishLine())
    done();
  else if (firstTrialFlag)
    solveMaze_FirstTrial();
  else 
    solveMaze_SecondTrial();
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

bool checkFinishLine() {
  for (int i = 0; i<3; i++)
    if (sensors[0+i] && !sensors[1+i] && sensors[2+i]){
      delay(40);
      readSensors();
      if (sensors[0+i] && !sensors[1+i] && sensors[2+i]) return true;
    }
  
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------


void readSensors() {

  //Reads the sensor values and stores them in the "sensors" array.
  
  sensors[0] = digitalRead(leftMost_sensor);
  sensors[1] = digitalRead(left_sensor);
  sensors[2] = digitalRead(center_sensor);
  sensors[3] = digitalRead(right_sensor);
  sensors[4] = digitalRead(rightMost_sensor);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void testSensors() {

  readSensors();
  Serial.println("LeftMost Sensor: " + (String)sensors[0]);
  Serial.println("Left Sensor: " + (String)sensors[1]);
  Serial.println("Center Sensor: " + (String)sensors[2]);
  Serial.println("Right Sensor: " + (String)sensors[3]);
  Serial.println("RightMost Sensor: " + (String)sensors[4]);
  
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

float computePosition(){

  //Computes a metric for the current orientation of the robot car.
  //If the car is to the right of the line, it will return a value between 0 and 2, depending on how deviated it is.
  //If the car is to the left of the line, it will return a value between 2 and 4, depending on how deviated it is.
  //If the car is aligned perfectly on the line, or if the car cannot detect a line, the function will return 2.

  //Serial.println("Test 1");
  if (!sensors[0] && !sensors[1] && !sensors[2] && !sensors[3] && !sensors[4]) return 2;
  //Serial.println("Test 2");
  return (0.0 * sensors[0] + 1.0 * sensors[1] + 2.0 * sensors[2] + 3.0 * sensors[3] + 4.0 * sensors[4]) /
      (sensors[0] + sensors[1] + sensors[2] + sensors[3] + sensors[4]);

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void spike() {
  set_AbsMotorSpeeds(spikeSpeed, spikeSpeed, spikeSpeed, spikeSpeed);
  delay(spikeDelay);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void stopCar() {
  analogWrite(leftMotors_ENA, 0);
  analogWrite(leftMotors_ENB, 0);
  
  analogWrite(rightMotors_ENA, 0);
  analogWrite(rightMotors_ENB, 0);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void set_AbsMotorSpeeds(int frontLeftSpeed, int backLeftSpeed, int frontRightSpeed, int backRightSpeed) {

  analogWrite(leftMotors_ENA, frontLeftSpeed);
  analogWrite(leftMotors_ENB, backLeftSpeed);

  analogWrite(rightMotors_ENA, backRightSpeed);
  analogWrite(rightMotors_ENB, frontRightSpeed);

}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void set_MotorSpeeds(int frontLeftSpeed, int backLeftSpeed, int frontRightSpeed, int backRightSpeed) { 

  //Sets the speeds of the left and right motors with the value in leftSpeed and rightSpeed.
  //If leftSpeed or rightSpeed are negative, the corresponding motor will move backwards.

  //Set motor directions:

  if (frontLeftSpeed >= 0){
    digitalWrite(leftMotors_IN1, LOW);
    digitalWrite(leftMotors_IN2, HIGH);
  } else{
    frontLeftSpeed *= -1;
    digitalWrite(leftMotors_IN1, HIGH);
    digitalWrite(leftMotors_IN2, LOW);
  }

  if (backLeftSpeed >= 0){
    digitalWrite(leftMotors_IN3, HIGH);
    digitalWrite(leftMotors_IN4, LOW);
  } else{
    backLeftSpeed *= -1;
    digitalWrite(leftMotors_IN3, LOW);
    digitalWrite(leftMotors_IN4, HIGH);
  }

  if (frontRightSpeed >= 0){
    digitalWrite(rightMotors_IN3, HIGH);
    digitalWrite(rightMotors_IN4, LOW);
  } else{
    frontRightSpeed *= -1;
    digitalWrite(rightMotors_IN3, LOW);
    digitalWrite(rightMotors_IN4, HIGH);
  }

  if (backRightSpeed >= 0){
    digitalWrite(rightMotors_IN1, HIGH);
    digitalWrite(rightMotors_IN2, LOW);
  } else{
    backRightSpeed *= -1;
    digitalWrite(rightMotors_IN1, LOW);
    digitalWrite(rightMotors_IN2, HIGH);
  }

  frontLeftSpeed = (frontLeftSpeed > maxSpeed) ? maxSpeed:frontLeftSpeed;
  backLeftSpeed = (backLeftSpeed > maxSpeed) ? maxSpeed:backLeftSpeed;
  frontRightSpeed = (frontRightSpeed > maxSpeed) ? maxSpeed:frontRightSpeed;
  backRightSpeed = (backRightSpeed > maxSpeed) ? maxSpeed:backRightSpeed;
  
  //Set motor speeds with the absolute value of the inputs:
  set_AbsMotorSpeeds(frontLeftSpeed, backLeftSpeed, frontRightSpeed, backRightSpeed);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void moveStraight() {

    //Uses PID control to stablizie the car, so that it moves straight on the line.
    currPosition = computePosition();
    error = (currPosition - 2.0);
    //Serial.println("Error = " + String(error));
    derivative = error - prevError;
    integral = integral + error;

    prevError = error;

    PIDvalue = error * Kp + integral * Ki + derivative * Kd;
    //Serial.println("PIDvalue = " + String(PIDvalue));

    set_MotorSpeeds(forwardSpeeds[0] + PIDvalue, forwardSpeeds[1] + PIDvalue, forwardSpeeds[2] - PIDvalue, forwardSpeeds[3] - PIDvalue);
    //Serial.println("Left Motor Speed = " + (String)(forwardSpeed + PIDvalue));
    //Serial.println("Right Motor Speed = " + (String)(forwardSpeed - PIDvalue + 20));
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------

void turnLeft(){

  //Serial.println("Test");
  delay(70);
  set_MotorSpeeds(0, 0, 0, 0);
  delay(1000);
  set_MotorSpeeds(-254, -254, 254, 254);
  delay(150);
  set_MotorSpeeds(turnLeftSpeeds[0], turnLeftSpeeds[1], turnLeftSpeeds[2], turnLeftSpeeds[3]);
  while(digitalRead(center_sensor));
  //Serial.println("Test Left Turning");
  //delay(50);
  while(!digitalRead(left_sensor));
  set_MotorSpeeds(0, 0, 0, 0);
  delay(1000);

  if (digitalRead(rightMost_sensor)) {
    set_MotorSpeeds(150, 150, -150, -150);
    delay(30);
  }
  
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void turnRight(){

   //Serial.println("Right Movement");
  //Turns the car to the right.
  set_MotorSpeeds(0, 0, 0, 0);
  delay(1000);
  set_MotorSpeeds(254, 254, -254, -254);
  delay(150);
  set_MotorSpeeds(turnRightSpeeds[0], turnRightSpeeds[1], turnRightSpeeds[2], turnRightSpeeds[3]);
  while(digitalRead(center_sensor));
  //Serial.println("Test Left Turning");
  //delay(50);
  while(!digitalRead(right_sensor));
  set_MotorSpeeds(0, 0, 0, 0);
  delay(1000);

  if (digitalRead(leftMost_sensor)) {
    set_MotorSpeeds(-150, -150, 150, 150);
    delay(30);
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

void turnAround(){
   //Serial.println("turn around");
  //Turns the car to the right.
  delay(100);
  set_MotorSpeeds(0, 0, 0, 0);
  delay(1000);
  set_MotorSpeeds(-254, -254, 254, 254);
  delay(150);
  set_MotorSpeeds(turnAroundSpeeds[0], turnAroundSpeeds[1], turnAroundSpeeds[2], turnAroundSpeeds[3]);
  while(digitalRead(center_sensor));
  //Serial.println("Test Left Turning");
  //delay(50);
  while(!digitalRead(left_sensor));
  set_MotorSpeeds(0, 0, 0, 0);
  delay(1000);

  if (digitalRead(rightMost_sensor)) {
    set_MotorSpeeds(150, 150, -150, -150);
    delay(30);
  }
  //delay(250);
  //set_MotorSpeeds(0, 0, 0, 0);
  //delay(1000);

  //while(!sensors[3] && !sensors[4]){
      //set_MotorSpeeds(0, 0, -254, -254);
      //readSensors();
  //}

  //while(sensors[0] || sensors[1] || sensors[2] || sensors[3] || sensors[4]){
    //set_MotorSpeeds(forwardSpeeds[0], forwardSpeeds[1], forwardSpeeds[2], forwardSpeeds[3]);
    //readSensors();
  //}
  
  //set_MotorSpeeds(0, 0, 0, 0);
  //delay(1000);
  
  //while(!digitalRead(center_sensor)){
    //set_MotorSpeeds(turnRightSpeeds[0], turnRightSpeeds[1], turnRightSpeeds[2], turnRightSpeeds[3]);
  //}

  //set_MotorSpeeds(0, 0, 0, 0);
  //delay(1000);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------

char simplifyDirections(char direction1, char direction2, char direction3) {

  //Substitutes a combination of 3 consecutive movements with 1 simpler movement.
  //For example, if the robot moves Left, then Backwards, then Right, then this is equivalent to just moving straight.
  //Will be used to ensure that the path stored in the paths array is the shortest path.
  
  if (direction1 == 'L' && direction2 == 'B' && direction3 == 'L') return 'S';
  if (direction1 == 'L' && direction2 == 'B' && direction3 == 'S') return 'R';
  if (direction1 == 'L' && direction2 == 'B' && direction3 == 'R') return 'B';
  
  if (direction1 == 'S' && direction2 == 'B' && direction3 == 'L') return 'R';
  if (direction1 == 'S' && direction2 == 'B' && direction3 == 'S') return 'B';
  if (direction1 == 'S' && direction2 == 'B' && direction3 == 'R') return 'L';

  if (direction1 == 'R' && direction2 == 'B' && direction3 == 'L') return 'B';
  if (direction1 == 'R' && direction2 == 'B' && direction3 == 'S') return 'L';
  if (direction1 == 'R' && direction2 == 'B' && direction3 == 'R') return 'S';

}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void printPath(){
  String pathString = "";
  for (int i = 0; i<pathLength; i++){
    pathString += path[i];
  }
  Serial.println(pathString);
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void updatePath(char newDirection)
{

  //Called when the robot passes any junction.
  //Takes as input the direction taken by the car to pass the junction, and adds it to the array.
  //If the function detects any possible simplifications before adding the new direction, it will simplify the array by using the
  //"simplifyDirections" function on the most recent 3 directions.
  leftFlag = false;
  rightFlag = false;
  unsigned long currTime = millis();
  
  if (currTime - prevTime < 500) {
    path[pathLength - 1] = newDirection;
    return;
  }
  prevTime = currTime;

  path[pathLength] = newDirection;
  pathLength++;
  printPath();
 
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void done() {
  pathIndex = 0;
  doneFlag = true;
  int newPathLength = 0;

  //Serial.println("Test");
  while (true) {
    
    int numOfShorts = 0;
    
    for (int i = 0; i < pathLength; i++) {

      if (i != (pathLength - 1) && path[i + 1] == 'B') {

        char newChar = simplifyDirections(path[i], path[i + 1], path[i + 2]);
        path[newPathLength] = newChar;
        i += 2;
        numOfShorts++;

      }
      else
        path[newPathLength] = path[i];

      newPathLength++;
    }
    
    pathLength = newPathLength;
    newPathLength = 0;
    if (numOfShorts == 0) break;
  }

  printPath();
  stopCar();
  firstTrialFlag = false;
  delay(30000);
  doneFlag = false;
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void analyzeJunction(){

  //When the robot enters a junction, it should decide its next move according to the following rule:
  //If a left turn is possible, take it.
  //Else, if going straight is possible, move forward.
  //Else, if a right turn is possible, take it.
  //Else, a dead end has been reached (or the maze is done). Hence, turn around.
  //Serial.println((String)rightFlag);
  if (moveLeftFirst){
    if (sensors[0]) {
      leftFlag = true;
      return;
    }
    
    if (leftFlag) {
      updatePath('L');
      turnLeft();
      return;
    }

    if (sensors[4]){
      rightFlag = true;
      //Serial.println((String)rightFlag);
      return;
    }

    delay(30);
    readSensors();
    if (sensors[1] || sensors[2] || sensors[3]) {
      updatePath('S');
      return;
    }
  
    if (rightFlag) {
      updatePath('R');
      turnRight();    
    } else {
      updatePath('B');
      turnAround();
    }
  } 
  else {
    if (sensors[4]) {
      rightFlag = true;
      return;
    }
    
    if (rightFlag) {
      //Serial.println((String)(millis() - junctionDetectionTime));
      updatePath('R');
      turnRight();
    }

    if (sensors[0]){
      leftFlag = true;
      //Serial.println((String)rightFlag);
      return;
    }

    delay(20);
    readSensors();
    if (sensors[1] || sensors[2] || sensors[3]) {
      updatePath('S');
      return;
    }
  
    if (leftFlag) {
      updatePath('L');
      turnLeft();    
    } else {
      updatePath('B');
      turnAround();
    }
  }
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void solveMaze_FirstTrial() {
  //If there is a junction or a dead end, analyze it and decide which direction to take.
  if(sensors[0] || sensors[4] || (!sensors[0] && !sensors[1] && !sensors[2] && !sensors[3] && !sensors[4]) || leftFlag || rightFlag) 
    analyzeJunction();                                                                      
  //Otherwise, move straight.
  else                                                                                           
    moveStraight();           
}

//---------------------------------------------------------------------------------------------------------------------------------------------------------------

void solveMaze_SecondTrial() {

  moveStraight();
  if (pathIndex == pathLength) {
    if (sensors[0] || sensors[4])
      done();
    return;
  }
  
  if (path[pathIndex] == 'L' && sensors[0]) {
      delay(100);
      turnLeft();
      pathIndex++;
  } else if (path[pathIndex] == 'R' && sensors[4]) {
      delay(100);
      turnRight();
      pathIndex++;
  } else if (path[pathIndex] == 'S' && (sensors[0] || sensors[4])) {
      set_MotorSpeeds(forwardSpeeds[0], forwardSpeeds[1], forwardSpeeds[2], forwardSpeeds[3]);
      delay(200);
      pathIndex++;
  }

}
