#include <Arduino.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(10, 11); // RX, TX

void setup(){
  Serial.begin(9600);
  mySerial.begin(115200);
}

void loop(){
  String data = "";
  int i = 0;

  for (i=0; i<3; i++){//enter
    char a = 10;
    mySerial.write(a);
  }

  char board1[10] = {'N','1',' ','v','1','0',10};//enter speed
  for (i=0; i<10; i++){
      char a = board1[i];
      mySerial.write(a);
  }
  Serial.print(board1);

  //////////////////////////////////////////////////////////////////
  char conf[10] = {'N','1',' ','O',' ','G','3',10}; //ask speed
  for (i=0; i<10; i++){
      char a = conf[i];
      mySerial.write(a);
  }
  Serial.print(conf);

  while (mySerial.available()) { //read speed in serial port
    data += mySerial.read(); 
  }
  Serial.print(">> "); Serial.print(data); Serial.println();
  //>> 78 49 32 118 49 48 10 (N1 v10)
  //>> 78 49 32 118 50 48 10 (N1 v20)
  //////////////////////////////////////////////////////////////////
  delay(5000);

  char clear[10] = {'N','1',' ','O',' ','C',10}; //clear errors
  for (i=0; i<10; i++){
      char a = clear[i];
      mySerial.write(a);
  }
}
