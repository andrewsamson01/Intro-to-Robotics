/* 
John Ripple and Andrew Samson
CSCI 442 Intro to Robotics
Lab 4: Solve the Maze
*/
// Include Libraries
#include <PinChangeInterrupt.h>
// If using any Ultrasonic Distance Sensors
#include <HCSR04.h>

////////////////////////////////////////////////////
// Copy constants and definitions from Lab 3
////////////////////////////////////////////////////

#define SHIELD true

//SHIELD Pin varables - cannot be changed
#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13

//Driver Pin variable - any 4 analog pins (marked with ~ on your board)
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6

// Defining these allows us to use letters in place of binary when
// controlling our motors
#define A 0
#define B 1
#define pushButton 2 // install a Pullup button with its output into Pin 2

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 40 //Gear ratio = 40/24 * (Encoder counts per disk=24)
#define DistancePerRev      25.1
#define DegreesPerRev       22.3

//These are to build your moves array, a la Lab 2
#define FORWARD             2
#define LEFT                1
#define RIGHT              -1

// these next two are the digital pins we'll use for the encoders
// You may change these as you see fit.
#define EncoderMotorLeft  7
#define EncoderMotorRight 8
#define Bumper 9

// Proportional Control constants
// what are your ratios of PWM:Encoder Count error?
#define GAIN_A 3
#define GAIN_B 3
// how many encoder counts from your goal are accepteable?
#define distTolerance 0

// minimum power settings
// Equal to the min PWM for your robot's wheels to move
// May be different per motor
#define deadband_A 80
#define deadband_B 80
bool bump = false;


// Define IR Distance sensor Pins
#define frontIR A0
#define sideIR  A1

// If using any Ultrasonic - change pins for your needs
#define trig 3
#define echo 6
// if side dist is Ultrasonic
HCSR04 hc(trig, new int[2] {echo, 9}, 2); //left
// if front dist is Ultrasonic

//SR04 frontUS = SR04(trig, echo);

// Define the distance tolerance that indicates a wall is present
#define wallTol 3 //cm



// Wheel class to store values and functions related to each wheel
class wheel{
  public:
  // Init function
  wheel(int pin, void *function){
    pinMode(pin, INPUT_PULLUP); //set the pin to input
    attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(pin), function, CHANGE);
    Serial.println("Here");
  }

  // Finds the displacement from the last time it was called.
  double displacement(){
    double tbr =(count - last_count) / EncoderCountsPerRev * DistancePerRev;
    last_count = count;  
    return tbr;
  }

  // Update encoder count based on direction moving
  void changeCount() {
    if (direct > 0) {
      count++;
    } else {
      count--;
    }
  }

  // Find the desired counts based on a desired distance to travel
  void countDesiredUpdate(int distance) {
    countsDesired = (EncoderCountsPerRev / DistancePerRev * distance)*direct + count;
  }

  // Class variables
  long int count = 0;
  int direct = 1;
  int last_count = 0;
  int countsDesired = 0;
  
  private:
};

void indexRightEncoderCount();
void indexLeftEncoderCount();
int drive(float distance, int ldir, int rdir);
void bumperContact();
void updatePosition();

// Create the wheel objects
wheel right(7, indexRightEncoderCount);
wheel left(8, indexLeftEncoderCount);


int moves[50]; // Empty array of 50 moves, probably more than needed, just in case

void setup() {
  // set stuff up
  Serial.begin(9600);
  motor_setup();
  pinMode(pushButton, INPUT_PULLUP);
  // add additional pinMode statements for any bump sensors
  

   // add additional pinMode statements for any bump sensors
   //GRD AND VCC FOR SENSORS AND ACTUATORS

  //right encoder
   pinMode(4, OUTPUT);
   digitalWrite(4, HIGH);

   //IR Sensor
   pinMode(A3, OUTPUT);
   digitalWrite(A3, LOW);

  //right Ultrasonic
   pinMode(A2, OUTPUT);
   analogWrite(A2, LOW);
   pinMode(A1, OUTPUT);
   analogWrite(A1, 1023);

  //left sensor 
   pinMode(A4, OUTPUT);
   analogWrite(A4, LOW);
   pinMode(A5, OUTPUT);
   analogWrite(A5, 1023);

   pinMode(A0, INPUT);
   //pinMode(A1, INPUT);
   
   // Attach ISR to bumper
//   Serial.print("Now setting up the Bumper: Pin ");
//   Serial.print(Bumper);
//   Serial.println();
//   pinMode(Bumper, INPUT_PULLUP);     //set the pin to input
//   attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Bumper), bumperContact, CHANGE);

} /////////////// end of setup ////////////////////////////////////

int last_time = 0;
/////////////////////// loop() ////////////////////////////////////
void loop()
{
  
//  while(true){
//    Serial.print(readFrontDist());
//    Serial.print('\t');
//    Serial.print(hc.dist(0));
//    Serial.print('\t');
//    Serial.println(hc.dist(1));
//    delay(60);
//  }
  while (digitalRead(pushButton) == 1); // wait for button push
  while (digitalRead(pushButton) == 0); // wait for button release
  explore();
  run_motor(A, 0);
  run_motor(B, 0);
  solve();
  while (1) { //Inifnite number of runs, so you don't have to re-explore everytime a mistake happens
    while (digitalRead(pushButton) == 1); // wait for button push
    while (digitalRead(pushButton) == 0); // wait for button release
    runMaze();
    run_motor(A, 0);
    run_motor(B, 0);
  }
}
////////////////////////////////////////////////////////////////////////////////
float readFrontDist() { 
  // If IR distance sensor
  int reading = analogRead(frontIR);
  float dist = 1/(0.1* reading * 5.0/1023 + -0.026) - 0.9;// Equation from your calibration;

  // if Ultrasonic
  // float dist = frontUS.Distance(); //(returns in cm)

  return dist;
}
////////////////////////////////////////////////////////////////////////////////
float readSideDist() {
  // If IR distance sensor
  int reading = analogRead(sideIR);
  float dist = 1/(0.122*reading*5.0/1023 + -0.109) - -3.5;// Equation from your calibration;

  // IF Ultrasonic
  // float dist = hc.Distance(); //(returns in cm)
  
  
  return dist;
}
//////////////////////////////// end of loop() /////////////////////////////////
void explore() {
  float sider, sidel,front;
  int i = 0;
  float fronttol = 10;
  double tur = 20*PI/4; // Arc length = r*theta ~~ 5*PI
  while (digitalRead(pushButton) == 1) { //while maze is not solved
    // Read distances
    if(millis() - last_time >= 60){
      sider = hc.dist(0);
      sidel = hc.dist(1);
      last_time = millis();
    }
    front = readFrontDist();
    
    if (sider > wallTol) {// If side is not a wall
      drive(tur * 0.9,1,-1);
      drive(33, 1, 1);
      //backup
      if(front < fronttol)
        drive( fronttol - front, -1, -1);
      moves[i] = RIGHT;
      moves[i+1] = FORWARD;
      i += 2;
      // turn and drive forward
      // Record actions
    }
    else if (front > wallTol) {// else if front is not a wall
      drive(33, 1, 1);
      //backup
      if(front < fronttol)
        drive( fronttol - front, -1, -1);
      // Record action
      moves[i] = FORWARD;
      i++;
    } else if(sidel > wallTol){
      drive(tur ,-1,1);
      drive(33, 1, 1);
      //backup
      if(front < fronttol)
        drive( fronttol - front, -1, -1);
      moves[i] = LEFT;
      moves[++i] = FORWARD;
      i++;
      // turn away from side
      // Record action
    }else{
      drive(tur * 2, 1, -1);
      moves[i] = RIGHT;
      moves[++i] = RIGHT;
      i++; 
      
    }
  }
}
////////////////////////////////////////////////////////////////////////////////
void solve() {
  // Write your own algorithm to solve the maze using the list of moves from explore
    for( int i = 0; i < 50; i++){
      if(moves[i] == RIGHT && moves[i+1] == RIGHT){
        int temp = 0;
        i += 2;
        while(moves[i + temp] == FORWARD){
          temp++;
        }
        for (int j = i - temp - 2; j < i + temp; j++) {
          moves[j] = 0;
        }
        i += temp;
        moves[i] *= -1;
      }
  }
}
////////////////////////////////////////////////////////////////////////////////
void runMaze() {
  int j = 0;
  int L = 0;
  int R = 0;
  for(int i = 0; i < sizeof(moves)/sizeof(moves[0]); i++){
    double tur = 20*PI/4; // Arc length = r*theta ~~ 5*PI
    if(moves[i]==LEFT){
      double vals[] = {1.0, 1.0};
      drive(tur *vals[L],-1,1);
      L++;
    }
    else if(moves[i]==RIGHT){
      double vals[] = {.9, .9, 0.9};
      drive(tur * vals[R],1,-1);
      R++;
    }
    else if(moves[i] == FORWARD){
      // Fill with code to drive forward
      double vals[] = {1.0, 1.0, 0.95, 2.8, 1.8, 1.0};
      double distance = 33 * vals[j]; // Need to change this to whatever the correct distance is
      drive(distance,1,1);
      j++;
    }
    run_motor(A, 0);
    run_motor(B, 0);
    //delay(1000);
  }
}
////////////////////////////////////////////////////////////////////////////////

int drive(float distance, int ldir, int rdir)
{
  // Update class variables
  right.direct = rdir;
  left.direct = ldir;
  right.countDesiredUpdate(distance);
  left.countDesiredUpdate(distance);
  // create variables needed for this function
  int cmdLeft, cmdRight, errorLeft, errorRight, lastError, lastTime, thisTime;
  double kd = 1.3;

  // Find the number of encoder counts based on the distance given, and the 
  // configuration of your encoders and wheels

  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorLeft = distTolerance + 1;
  errorRight =  right.countsDesired + 1;
  lastError = errorRight+1;

  // Begin PD control until move is complete
  while ((errorLeft > distTolerance || errorRight > distTolerance) && (lastError >= errorRight))
  {
    // React to the bump ISR being triggered
    if (bump) {
      bump = false;
      long int encoders[] = {left.count, right.count, left.countsDesired, right.countsDesired}; // Store globals before the bump
      drive(5, -1, -1); // Reverse, turn, and go forward again
      drive(2, 1, -1);
      drive(5, 1, 1);
      // Restore global counts before the bump
      left.count = encoders[0];
      right.count = encoders[1];
      left.countsDesired = encoders[2];
      right.countsDesired = encoders[3];
    }
    // Constants to make the right and left motor errors equal
    double expGain = 9.0;
    int diff = errorRight - errorLeft;
    //update_position();

    // Motor control based on PD controls
    thisTime = millis();
    cmdLeft = computeCommand(GAIN_A, deadband_A, errorLeft) + kd*(lastError - errorLeft) / (thisTime- lastTime);
    cmdRight = computeCommand(GAIN_B, deadband_B, errorRight)* abs(expGain - diff)/ expGain+ kd*(lastError - errorRight) / (thisTime- lastTime);

    // Set new PWMs
    run_motor(A, cmdLeft * ldir);
    run_motor(B, cmdRight * rdir);

    // Update encoder error
    Serial.print(errorLeft);
    lastError = errorRight;
    errorLeft = abs(left.countsDesired - left.count);
    errorRight = abs(right.countsDesired - right.count) ;
    lastTime = thisTime;
    
  }

}
//////////////////////////////////////////////////////////
int computeCommand(int gain, int deadband, int error)
//  gain, deadband, and error, both are integer values
{
  if (error <= distTolerance) { // if error is acceptable, PWM = 0
    return (0);
  }

  int cmdDir = (gain * error); // Proportional control
  cmdDir = constrain(cmdDir,deadband,220); // Bind value between motor's min and max
  return(cmdDir);
}

// These are the encoder interupt funcitons, they should NOT be edited
void indexLeftEncoderCount()
{
  left.changeCount();
}
//////////////////////////////////////////////////////////
void indexRightEncoderCount()
{
  right.changeCount();
}

///////////////////////////////////////////////////////////
void bumperContact(){
  if (!digitalRead(Bumper)) {
    bump = true;
  }
}
