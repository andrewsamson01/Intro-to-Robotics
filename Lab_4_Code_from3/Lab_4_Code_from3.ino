/*
John Ripple and Andrew Samson
2/24/2023
Lab 3: Closed Loop Control
MEGN 441: Intro to Robotics
*/
// Include Libraries
#include <PinChangeInterrupt.h>
#include <HCSR04.h> // If using any Ultrasonic Distance Sensors

// If using any Ultrasonic - change pins for your needs
#define trig 10
#define echoLeft 9
#define echoRight 6
#define echoFront 4

// if side sensor is Ultrasonic
HCSR04 hc(trig, new int[3] {echoRight, echoLeft, echoFront}, 3);
// if front sensor is Ultrasonic
//HCSR04 frontUS(trig, echo);

// Define the distance tolerance that indicates a wall is present
#define wallTol 13 //cm

//int moves[50]; // Empty array of 50 moves, probably more than needed, just in case






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
#define deadband_A 100
#define deadband_B 100
bool bump = false;

//pos
double x = 0;
double y = 0;
double phi = 0;
// Lab specific variables

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

//Sensor Avg stuff
int sensorReadings = 0;
float leftSensorSum = 0;
float rightSensorSum = 0;
float frontSensorSum = 0;


// Create the wheel objects
wheel right(7, indexRightEncoderCount);
wheel left(8, indexLeftEncoderCount);
int moves[50]; // Fill in this array will forward distances and turn directions in the maze (a la Lab 2)
int last_time = 0;
void setup() {
  // set stuff up
  Serial.begin(9600);
  motor_setup();
  pinMode(pushButton, INPUT_PULLUP);
  // add additional pinMode statements for any bump sensors
  

   // add additional pinMode statements for any bump sensors
   //GRD AND VCC FOR SENSORS AND ACTUATORS

  //right encoder 5v
   pinMode(5, OUTPUT);
   digitalWrite(5, HIGH);
  
  //Front ultrasonic
   pinMode(A0, OUTPUT);
   analogWrite(A0, 1023);

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

   //pinMode(4, INPUT);
   //pinMode(A1, INPUT);
   

  
} /////////////// end of setup ////////////////////////////////////

/////////////////////// loop() ////////////////////////////////////
void loop()
{
    
  while (digitalRead(pushButton) == 1){
    printSensorData();
    }
  while (digitalRead(pushButton) == 0); // wait for button release
  explore();
  run_motor(A, 0);
  run_motor(B, 0);
  while (digitalRead(pushButton) == 0);
  solve();
  while (true) { //Inifnite number of runs, so you don't have to re-explore everytime a mistake happens
    while (digitalRead(pushButton) == 1); // wait for button push
    while (digitalRead(pushButton) == 0); // wait for button release
    runMaze();
    run_motor(A, 0);
    run_motor(B, 0);
  }
  
}
//////////////////////////////// end of loop() /////////////////////////////////


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
    
    // Constants to make the right and left motor errors equal
    double expGain = 9.0;
    double diff = errorRight - errorLeft;
    //update_position();

    // Motor control based on PD controls
    thisTime = millis();
    cmdLeft = computeCommand(GAIN_A, deadband_A, errorLeft);
    cmdRight = computeCommand(GAIN_B, deadband_B, errorRight)* abs(expGain - diff)/ expGain;

    // Set new PWMs
    run_motor(A, cmdLeft * ldir);
    run_motor(B, cmdRight * rdir);

    // Update encoder error
    //Serial.print(errorRight);
    delay(2);
    lastError = errorRight;
    errorLeft = abs(left.countsDesired - left.count);
    errorRight = abs(right.countsDesired - right.count);
//    Serial.println(errorRight);
//    Serial.println(errorLeft);
    lastTime = thisTime;
    
  }
    run_motor(A, 0);
    run_motor(B, 0);
}
////////////////////////////////////////////////////////
float readFrontDist() { 
  // If IR distance sensor
  int reading = analogRead(A0);
  float dist = 1/(0.1* reading * 5.0/1023 + -0.026) - 0.9;// Equation from your calibration;

  // if Ultrasonic
  // float dist = frontUS.dist(); //(returns in cm)

  return dist;
}



void explore() {
  double sider, sidel,front;
  int i = 0;
  float fronttol = 10;
  double tur = 20*PI/4.0; // Arc length = r*theta ~~ 5*PI
  int itters = 0;
  float IrAvg = 0;
  float centimetersForward = 29;
  while (digitalRead(pushButton) == 1) { //while maze is not solved
    // Read distances
    //printSensorData();
    sider = 0;
    sidel = 0;
    front = 0;
    
    // if(millis() - last_time >= 60){
    for(int j = 0; j < 5; j++){
      sider += hc.dist(0);
      sidel += hc.dist(1);    
      last_time = millis();
      front += hc.dist(2);
      delay(60);
      Serial.print("F: ");
    Serial.print(front);
    Serial.print("\tR: ");
    Serial.print(sider);
    Serial.print("\tL: ");
    Serial.println(sidel);
      
     }
     sider /= 5.0;
     sidel /= 5.0;
     front /= 5.0;
    
    Serial.print("AvgF: ");
    Serial.print(front);
    Serial.print("\tAvgR: ");
    Serial.print(sider);
    Serial.print("\tAvgL: ");
    Serial.println(sidel);
    
     if (sider > wallTol) {// If side is not a wall
       Serial.println("TurnRt");
       drive(tur * 0.9, 1,-1);
       Serial.println("ThenStraight");
       drive(centimetersForward, 1, 1);
       moves[i] = RIGHT;
       moves[i+1] = FORWARD;
       i += 2;
       
       // turn and drive forward
       // Record actions
     }
     else if (front > fronttol || front < 2.0) {// else if front is not a wall
       Serial.println("Strgt");
       drive(centimetersForward, 1, 1);
       //Reverse
//       if (front < fronttol && front > 2.0) {
//        drive(fronttol - front, -1, -1);
//       }
       // Record action
       moves[i] = FORWARD;
       i++;
     } else {
       Serial.println("TurnLt");
       drive(tur ,-1,1);
       //drive(centimetersForward, 1, 1);
       moves[i] = LEFT;
       //moves[++i] = FORWARD;
       i++;
       
       // turn away from side
       // Record action
     }
//     else{
//       Serial.println("AbtFce");
//       drive(tur * 2, 1, -1);
//       moves[i] = RIGHT;
//       moves[++i] = RIGHT;
//       i++; 
//      
//     }
  }
}



void solve() {
// Write your own algorithm to solve the maze using the list of moves from explore
    int LEFT = 1;
    int RIGHT = -1;
    int FORWARD = 2;
    int moves[50] = {FORWARD, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT, FORWARD, LEFT, LEFT, FORWARD, RIGHT, FORWARD, RIGHT, FORWARD, LEFT};
    for( int i = 0; i < 50; i++){
      if(moves[i] == LEFT && moves[i+1] == LEFT){
        int temp = 0;
        i += 2;
        while(moves[i + temp] == FORWARD || (moves[i+temp] == -moves[i- temp - 3])){
          temp++;
        }
        cout << i - temp - 4 << endl;
        if(moves[i - temp - 4] == RIGHT && moves[i + temp ] == RIGHT)
        {
            moves[i - temp - 4] = FORWARD;
        }
        for (int j = i - temp - 3; j < i + temp; j++) {
          moves[j] = 0;
        }
        i += temp;
      }
    }
        for( int i = 0; i < 50; i++){
            cout << moves[i] << ' ';
        }
    cout << moves;
    return 0;
 }


void runMaze() {
  int j = 0;
  int L = 0;
  int R = 0;
  for (int i = 0; i < sizeof(moves)/sizeof(moves[0]); i++) { // Loop through entire moves list
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
    else{
      // Fill with code to drive forward
      double vals[] = {1.0, 1.0, 0.95, 2.8, 1.8, 1.0};
      double distance = 33 * vals[j]; // Need to change this to whatever the correct distance is
      drive(distance,1,1);
      j++;
    }
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


//////////////////////////////////////////////////////////

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

void update_position(){ //Updates position for localization
  double d_r = right.displacement(); //sets a constant position throughout function
  double d_l = left.displacement();

  double xOld = x;
  double yOld = y;
  x = cos(phi)*(d_r + d_l) / 2; //updates x postion
  y = sin(phi)*(d_r + d_l) / 2; //updates y position
   
  phi = (right.count - left.count) * 2.0*PI /EncoderCountsPerRev * 8 / 19; //updates orientation
  x = x + xOld;
  y = y + yOld;

  Serial.print("Left\t");
  Serial.print(left.count); 
  Serial.print("\t\tRight\t");
  Serial.print(right.count);
  Serial.print("\t   ");
  Serial.print(phi);
  Serial.print('\t');
  Serial.print(x);
  Serial.print('\t');
  Serial.println(y);
}




void printSensorData() {
    Serial.print("lEnc: ");
    Serial.print(right.count);
    Serial.print("\trEnc: ");
    Serial.print(left.count);
    Serial.print("\tfDist: ");
    Serial.print(hc.dist(2));
    Serial.print("\trDist: ");
    Serial.print(hc.dist(0));
    Serial.print("\tlDist: ");
    Serial.println(hc.dist(1));
}
