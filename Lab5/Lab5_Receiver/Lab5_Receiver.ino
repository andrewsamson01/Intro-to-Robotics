#include<SPI.h>                   // spi library for connecting nrf
#include<RF24.h>                  // nrf library
#include <nRF24L01.h>
RF24 radio(7, 8); // CE, CSN for Arduino Mega
//RF24 radio(9k, 10); // CE, CSN for Arduino UNO


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
void setup() {
  motor_setup();

  while (!Serial) ;
  Serial.begin(115200) ;     // start serial monitor baud rate
  Serial.println("Starting.. Setting Up.. Radio on..") ; // debug message
  radio.begin();        // start radio at ce csn pin 9 and 10
  radio.setPALevel(RF24_PA_MIN) ;   // set power level
  radio.setChannel(0x76) ;            // set chanel at 76
  const uint64_t pipe = 0xE0E0F1F1E0LL ;    // pipe address same as sender i.e. raspberry pi
  //const uint64_t pipe = 0x3130303030;
  radio.openReadingPipe(0, pipe);   //Setting the address at which we will receive the data
  
}
void loop() {
  radio.startListening();              //This sets the module as receiver
  char receivedMessage[32] = {0} ;   // set incmng message for 32 bytes
  if (radio.available()) {       // check if message is coming
    radio.read(receivedMessage, sizeof(receivedMessage));    // read the message and save
    //Serial.println(receivedMessage) ;    // print message on serial monitor 
    String data = String(receivedMessage);
    int count = 0;
    int start = 0;
    int arrayOfInputs[2] = {0};
    for(int i = 0; i < data.length(); i++){
      if(data[i] == ' ') {
        arrayOfInputs[count] = data.substring(start, i).toInt();
        start = i;
        count++;
      }
    }
    Serial.println(arrayOfInputs[0]);
    Serial.println(arrayOfInputs[1]);
    run_motor(A, arrayOfInputs[0]);
    run_motor(B, arrayOfInputs[1]);
    Serial.println("Turning off the radio.") ;   // print message on serial monitor
    radio.stopListening() ;   // stop listening radio
  }
  delay(10);
}

  // uint64_t newValue = *(uint64_t *)address;
  // newValue <<= 16; // shift out the garbage bytes, get zero's in.
  // newValue >>= 16; // and shift back again.
  // Serial.print((uint32_t)((newValue >> 32) & 0xFFFFFFFF), HEX);
  // Serial.print((uint32_t)(newValue & 0xFFFFFFFF), HEX);