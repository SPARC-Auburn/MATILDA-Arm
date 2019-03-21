/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo and Micro support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>


SoftwareSerial mySerial(10, 2); // RX, TX
SoftwareSerial mySerial2(A4, A5); // RX, TX

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);

  pinMode(7, OUTPUT); // Output for Brake 1
  pinMode(6, OUTPUT); // Output for Brake 2
  
  pinMode(5, OUTPUT);
  pinMode(3, OUTPUT);
  
  digitalWrite(5,HIGH);
  digitalWrite(3,HIGH);
  
  digitalWrite(7,HIGH);
  digitalWrite(6,HIGH);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  mySerial2.begin(9600);
  delay(2000);
}

void loop() { // run over and over
    while(Serial.available()){
      int a = Serial.read();
      while(!Serial.available()){}
      int b = (Serial.read()-127)/2;
      switch(a){
        case 1:
          mySerial.write(b+64);

          break;
        case 2:
          mySerial.write(b+192);
          break;
        case 3:
          mySerial2.write(b+64);    
          break;
        }  
    }
}

