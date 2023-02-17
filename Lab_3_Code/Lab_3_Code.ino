/* a closed loop proportional control
   ms 20200926
*/
// Include Libraries
#include <PinChangeInterrupt.h>

// Driver definitions

// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
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

// Lab Specific definitions

// Defining these allows us to use letters in place of binary when
// controlling our motors
#define A 0
#define B 1
#define pushButton 2 // install a Pullup button with its output into Pin 2
/* If you'd like to use additional buttons as bump sensors, define their pins 
 *  as descriptive names, such as bumperLeft etc. 
 */

// Drive constants - dependent on robot configuration
#define EncoderCountsPerRev 38.4
#define DistancePerRev      25.1
#define DegreesPerRev       22.3

//These are to build your moves array, a la Lab 2
#define FORWARD             0
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
#define distTolerance 3 

// minimum power settings
// Equal to the min PWM for your robot's wheels to move
// May be different per motor
#define deadband_A 140
#define deadband_B 140
bool bump = false;


// Lab specific variables
volatile unsigned int leftEncoderCount = 0;
volatile unsigned int rightEncoderCount = 0;
int moves[] = {FORWARD, LEFT, FORWARD, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD}; // Fill in this array will forward distances and turn directions in the maze (a la Lab 2)
//int moves[] = {FORWARD};
//int moves[] = {LEFT};
void setup() {
  // set stuff up
  Serial.begin(9600);
  motor_setup();
  pinMode(pushButton, INPUT_PULLUP);
  
  // add additional pinMode statements for any bump sensors
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);
  pinMode(10, OUTPUT);
  digitalWrite(10, LOW);
  
  Serial.print("Now setting up the Bumper: Pin ");
  Serial.print(Bumper);
  Serial.println();
  pinMode(Bumper, INPUT_PULLUP);     //set the pin to input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(Bumper), bumperContact, CHANGE);
  

  // Attaching Wheel Encoder Interrupts
  Serial.print("Encoder Testing Program ");
  Serial.print("Now setting up the Left Encoder: Pin ");
  Serial.print(EncoderMotorLeft);
  Serial.println();
  pinMode(EncoderMotorLeft, INPUT_PULLUP); //set the pin to input
  // this next line setup the PinChange Interrupt
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorLeft), indexLeftEncoderCount, CHANGE);
  // if you "really" want to know what's going on read the PinChange.h file :)
  /////////////////////////////////////////////////
  Serial.print("Now setting up the Right Encoder: Pin ");
  Serial.print(EncoderMotorRight);
  Serial.println();
  pinMode(EncoderMotorRight, INPUT_PULLUP);     //set the pin to input
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(EncoderMotorRight), indexRightEncoderCount, CHANGE);

  
} /////////////// end of setup ////////////////////////////////////

/////////////////////// loop() ////////////////////////////////////
void loop()
{
  
//  while(true){
//    Serial.print("Left\t");
//    Serial.print(leftEncoderCount);
//    Serial.print("\tRight\t");
//    Serial.println(rightEncoderCount); 
//  
//    if (digitalRead(pushButton) != 1){ // wait for button push
//      run_motor(A, 200);
//      run_motor(B, 200);
//      
//    }else{
//      run_motor(A, 0);
//      run_motor(B, 0);
//    }
//  }
int j = 0;
  while (digitalRead(pushButton) == 1);
  while (digitalRead(pushButton) == 0); // wait for button release
  for (int i = 0; i < sizeof(moves)/sizeof(moves[0]); i++) { // Loop through entire moves list
    double tur = 10.9; //5*PI/2
    if(moves[i]==LEFT){
      drive(tur,-1,1);
    }
    else if(moves[i]==RIGHT){
      drive(tur,1,-1);
    }
    else{
      // Fill with code to drive forward
      double vals[] = {1.0, 1.0, 0.95, 3.0, 2.0, 1.0};
      double distance = 27 * vals[j]; // Need to change this to whatever the correct distance is
      drive(distance,1,1);
      j++;
    }
    run_motor(A, 0);
    run_motor(B, 0);
    delay(1000);
  }
}
//////////////////////////////// end of loop() /////////////////////////////////


////////////////////////////////////////////////////////////////////////////////

int drive(float distance, int ldir, int rdir)
{
  // create variables needed for this function
  int countsDesired, cmdLeft, cmdRight, errorLeft, errorRight;
  countsDesired = EncoderCountsPerRev / DistancePerRev * distance;

  // Find the number of encoder counts based on the distance given, and the 
  // configuration of your encoders and wheels
  countsDesired = EncoderCountsPerRev / DistancePerRev * distance;

  // reset the current encoder counts
  leftEncoderCount = 0;
  rightEncoderCount = 0;
  
  // we make the errors greater than our tolerance so our first test gets us into the loop
  errorLeft = distTolerance + 1;
  errorRight =  distTolerance + 1;

  // Begin PID control until move is complete
  while (errorLeft > distTolerance || errorRight > distTolerance)
  {
    //noInterrupts();
    if (bump) {
      bump = false;
      unsigned int encoders[] = {leftEncoderCount, rightEncoderCount};
      drive(5, -1, -1);
      drive(3, 1, -1);
      drive(5, 1, 1);
      leftEncoderCount = encoders[0];
      rightEncoderCount = encoders[1];
    }
    double expGain = 9.0;
    int diff = rightEncoderCount - leftEncoderCount;
    Serial.print("Left\t");
    Serial.print(leftEncoderCount); 
    Serial.print("\t\tRight\t");
    Serial.println(rightEncoderCount);
    Serial.print("\t   ");
    
    Serial.println((diff));
    // according to the PID formula, what should the current PWMs be?
    
    cmdLeft = computeCommand(GAIN_A, deadband_A, errorLeft);
    cmdRight = computeCommand(GAIN_B, deadband_B, errorRight)* abs(expGain - diff)/ expGain;
    //interrupts();
    // Set new PWMs
    run_motor(A, cmdLeft * ldir);
    run_motor(B, cmdRight * rdir);

    // Update encoder error
    errorLeft = abs(countsDesired - leftEncoderCount);
    errorRight = abs(countsDesired - rightEncoderCount) ;
    //Serial.println(errorLeft);
    // If using bump sensors, check here for collisions
    // and call correction function

    /* Some print statements, for debugging
    Serial.print(errorLeft);
    Serial.print(" ");
    Serial.print(cmdLeft);
    Serial.print("\t");
    Serial.print(errorRight);
    Serial.print(" ");
    Serial.println(cmdRight);*/

  }

}
////////////////////////////////////////////////////////////////////////////////


// Write a function for turning with PID control, similar to the drive function


//////////////////////////////////////////////////////////


// If you want, write a function to correct your path due 
// to an active bump sensor. You will want to compensate somehow 
// for any wheel encoder counts that happend during this manuever


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


//////////////////////////////////////////////////////////

// These are the encoder interupt funcitons, they should NOT be edited

void indexLeftEncoderCount()
{
  leftEncoderCount++;
  
}
//////////////////////////////////////////////////////////
void indexRightEncoderCount()
{
  rightEncoderCount++;
}
///////////////////////////////////////////////////////////
void bumperContact(){
  if (!digitalRead(Bumper)) {
    bump = true;
  }
}
