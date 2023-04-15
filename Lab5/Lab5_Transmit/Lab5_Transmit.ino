#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8); // CE, CSN         


void setup() {
  Serial.begin(115200);
  radio.begin();        // start radio at ce csn pin 9 and 10
  radio.setPALevel(RF24_PA_MIN) ;   // set power level
  radio.setChannel(0x76) ;            // set chanel at 76
  const uint64_t pipe = 0xE0E0F1F1E0LL ;    // pipe address same as sender i.e. raspberry pi
  //const uint64_t pipe = 0x3130303030;
  radio.openWritingPipe(pipe);   //Setting the address at which we will receive the data
  radio.stopListening();
}

void loop() {
    if (Serial.available() > 0) {
    receiveData();
  }

}


void receiveData() {
  String data = Serial.readStringUntil('\n');
  int count = 0;
  int start = 0;
  int arrayOfInputs[2] = {0};
  Serial.print("You Sent me: ");
  for(int i = 0; i < data.length(); i++){
    if(data[i] == ' ') {
      arrayOfInputs[count] = data.substring(start, i).toInt();
      start = i;
      count++;
    }
  }
  char text[data.length() + 1];
  data.toCharArray(text, data.length()+1);
  radio.write(&text, sizeof(text)); 

  Serial.println(data); 
}