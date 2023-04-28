//This code Transmits data across wifi to be recieved by the other arduino via the serial connection
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
    if (Serial.available() > 0) { //If We have new data, interpret and send it
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
      arrayOfInputs[count] = data.substring(start, i).toInt(); //Converts the string that is read in into an integer
      start = i;
      count++;
    }
  }
  char text[data.length() + 1];
  data.toCharArray(text, data.length()+1); //Converts the int into something compatable with the library: a character array
  radio.write(&text, sizeof(text)); //Ships that signal out for the robot to recieve

  Serial.println(data); 
}
