
/* Dead Reckoning
  20220127 - gknave
*/

// Complete Tasks in Initial_Robot_Testing.ino first!!

/* Program TODO LIST
  1) Change the milliSecondsPerCM constant to the value you found in testing
  2) Change the milliSecondsPer90Deg constant to the value you found in testing
  3) Change the PWM of the forward function to the values you found in testing
  4) Write code for the button pause functionality
  5) Write your own turn function

  Note: type // to make a single line comment
        Comments are for the future you to understand how you wrote your code
*/

// If you have a kit with the moto shield, set this to true
// If you have the Dual H-Bridge controller w/o the shield, set to false
#define SHIELD true

// Defining these allows us to use letters in place of binary when
// controlling our motor(s)
#define A 0
#define B 1

//SHIELD Pin varables - cannot be changed
#define motorApwm 3
#define motorAdir 12
#define motorBpwm 11
#define motorBdir 13

// Dual H-Bridge motor controller pin variables - can be any 4 analog pins (marked with ~ on your board)
// Only used if SHIELD is false.
#define IN1 9
#define IN2 10
#define IN3 5
#define IN4 6


// Preprocessor Definitions
#define FORWARD    0
#define LEFT       1
#define RIGHT     -1
#define pushButton 2

// the following converts centimeters into milliseconds as long datatype
#define milliSecondsPerCM    40  //CHANGE THIS ACCORDING TO YOUR BOT
#define milliSecondsPer90Deg 860 //CHANGE THIS ACCORDNING TO YOUR BOT
#define PWM_A 117 //CHANGE THIS TO GET YOUR BOT TO DRIVE STRAIGHT
#define PWM_B 100 //CHANGE THIS TO GET YOUR BOT TO DRIVE STRAIGHT

// the itemized list of moves for the robot as a 1D array
// this setup assumes that all the turns are 90 degrees and that all motions are pairs of drives and turns.
int moves[] = {140, LEFT, 90, RIGHT, 60, RIGHT, 180, RIGHT, 100, LEFT, 60, RIGHT, 100, RIGHT, 150, RIGHT};

void setup() {
  // set up the motor drive ports
  motor_setup();
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT_PULLUP);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  int i, dist, dir;
  long time;
  while(digitalRead(pushButton) == 1){
      delay(1);
  }
  int turns[] = {-1, 60, 180, 1, 30, 180, 1, 30, 180, -1, 60, 180};
  for(i = 0; i < 12; i+=3){
    time = Arc(turns[i], turns[i+1], turns[i+2]);
  }

  while(digitalRead(pushButton) == 1){
      delay(1);
  }
  //This for loop steps (or iterates) through the array 'moves'
  for (i = 0; i < sizeof(moves) / 2 ; i = i + 2) {

    /* Write code here to make the program pause until
       the button has been pressed. Remember that getting a value
       from the board is digitalRead(pinNumber) and if statements
       need a double equals sign (==) for comparisons
    */

    delay(250);
    //Forward Leg of each step
    Serial.print("Step #:");
    Serial.println(i);
    dist = moves[i];
    Serial.print("Forward for");
    time = Forward(dist);
    Serial.print(time);
    Serial.println(" ms");
    delay(1000);

    //Turn Leg of each step
    Serial.print("Step #:");
    Serial.println(i + 1);
    dir = moves[i + 1];
    if (dir == LEFT) {
      time = Turn(90);
      Serial.print("turning LEFT ");
      Serial.print(time);
      Serial.println(" ms");
    }
    else {
      time = Turn(-90);
      Serial.println("turning RIGHT ");

      Serial.print(time);
      Serial.println(" ms");
    } // end of else motions conditional
    delay(1000);

  } // end of for loop
  Serial.println("That's All Folks!");
  delay(1000);
  exit(i);
} // the end

//////////////////////////////////////////////
unsigned long Forward(int distance) {
  unsigned long t;
  t = distance * milliSecondsPerCM; //Time to keep motors on

  //To drive forward, motors go in the same direction
  run_motor(A, PWM_A); //change PWM to your calibrations
  run_motor(B, PWM_B); //change PWM to your calibrations
  delay(t);
  run_motor(A, -10);
  run_motor(B, -10);
  delay(20);
  
  run_motor(A, 0);
  run_motor(B, 0);
  return (t);
}

//////////////////////////////////////////////
unsigned long Turn(int degrees) {
  unsigned long t;
  int sign = degrees / abs(degrees); //Find if left or right
  t = (abs(degrees) / 90) * milliSecondsPer90Deg; //Time to keep motors on

  // The run motor command takes in a PWM value from -255 (full reverse) to 255 (full forward)
  /* Using the Forward function as a guide,
   * Write commands in this Turn function to power the
   * motors in opposite directions for the calculated time
   * and then shut off
   */
   run_motor(A, PWM_A * -sign);
   run_motor(B, PWM_B * sign);
   delay(t);
   run_motor(A, 0);
   run_motor(B, 0);
   return(t);
}

unsigned long Arc(int direct, int radius, int degree){
  int len = 20;
  if(direct <  0){ //positive is counterclockwise, negative clockwise
    int a = constrain(PWM_B * (2*radius + len)/(2*radius - len) , 0, 255);
    run_motor(A, a);
    run_motor(B, PWM_B);
    Serial.print(a);
    Serial.print('\t');
    Serial.print(PWM_B);
  }else{//counterclockwise
    int b = constrain(PWM_A * (2*radius + len)/(2*radius - len), 0, 255);
    run_motor(A, PWM_A);
    run_motor(B, b);
    Serial.print(PWM_A);
    Serial.print('\t');
    Serial.print(b);
  }
  //if we assume the inside wheel has the same speed as straight then we can caluclate time based on circumfirence

  float in_travel = 2* PI * (radius - len / 2);
  unsigned long t;
  t = in_travel * milliSecondsPerCM * degree/360; //This was me being lazy and not wanting to measure loop time.
  delay(t);
  Serial.print(" Time: ");
  Serial.println(t);
  run_motor(A, 0);
  run_motor(B, 0);
  delay(500);
  return(t);
}
