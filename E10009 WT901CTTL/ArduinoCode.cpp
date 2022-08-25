#include <Arduino.h>
#include <Wire.h>
#include <JY901.h>

/*
Test on Uno R3.
JY901   UnoR3
TX <---> 0(Rx)
*/
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  //print received data. Data was received in serialEvent;
  Serial.print("Time:20");Serial.print(JY901.stcTime.ucYear);Serial.print("-");Serial.print(JY901.stcTime.ucMonth);Serial.print("-");Serial.println(JY901.stcTime.ucDay);

  Serial.print("Angle:");Serial.print((float)JY901.stcAngle.Angle[0]/32768*180);Serial.print(" ");Serial.print((float)JY901.stcAngle.Angle[1]/32768*180);Serial.print(" ");Serial.println((float)JY901.stcAngle.Angle[2]/32768*180);

  Serial.println("");
  delay(500);
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() 
{
  while (Serial.available()) 
  {
    JY901.CopeSerialData(Serial.read()); //Call JY901 data cope function
  }
}


