#include <Arduino.h>

#define ENCA 2
#define ENCB 3

#define RAD_S 0
#define DEG_S 1

void EncISR();
double EncReadSpeed(bool _mode, int ppr_enc);
void MatlabCode(float voltage, int ppr_enc, int cant_data);

//ENCODER                //**********************************************//
volatile double _t = 0;
volatile double _tant = 0;
volatile double _periodo = 1000000; 
volatile double _p_act = 20; 
volatile double _p_ant = 20; 
int count = 0;

void setup(){
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), EncISR, RISING);
}

void loop(){
  delay(5000);
  MatlabCode(12, 500, 1000); //voltage, ppr, cant_data  
}

void EncISR(){
    _t = micros();
    _p_act = _t - _tant;

    //El cambio máximo de velocidad es igual 20
    if(abs((_p_ant-_p_act)/(_p_ant*_p_act))<=0.000053){   
        _periodo=_p_act;
    }
    _p_ant=_p_act;
    _tant = _t;

    if(digitalRead(ENCB) == LOW){
        _periodo*=-1;
    }
    count++;
    if (count=501) count = 1;
}
double EncReadSpeed(bool _mode, int ppr_enc){
  double speed = 0;

  if(_mode){// deg/s
    speed = 360*1000000/(ppr_enc*_periodo);
  }
  else{ // rad/s
    speed = 2*PI*1000000/(ppr_enc*_periodo);
  }
    
  //delay(1);
  return speed;
}
void MatlabCode(float voltage, int ppr_enc, int cant_data){
  Serial.println("clc;");
  Serial.println("clear;");
  Serial.println("close all;"); Serial.println();
  
  Serial.print("vel = [ ");
  for(int i = 0; i<cant_data; i++){
    Serial.print(EncReadSpeed(0, ppr_enc));
    if(i < cant_data-1) Serial.print(", ");
  }
  Serial.println("];");
  Serial.println("t = [0:1:length(vel)-1];"); Serial.println();

  Serial.println("plot(t, vel, \"r\");");
  Serial.println("legend(\"vel\");");
  while(1);
}