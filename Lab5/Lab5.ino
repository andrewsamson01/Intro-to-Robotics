#include<SPI.h>                   // spi library for connecting nrf
#include<RF24.h>                  // nrf library
#include <nRF24L01.h>
RF24 radio(7, 8); // CE, CSN for Arduino Mega
//RF24 radio(9k, 10); // CE, CSN for Arduino UNO

void setup() {
  while (!Serial) ;
  Serial.begin(9600) ;     // start serial monitor baud rate
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
    Serial.println(receivedMessage) ;    // print message on serial monitor 
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