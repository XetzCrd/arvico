#include <RBot.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RoboClaw.h"
#include "ICM_20948.h" 

//Roboclaw
//SoftwareSerial serial(10,11);// RX, TX
RoboClaw roboclaw(&Serial3,10000);
//IMU
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
//Inclinómetro

//CONFIG se actualiza dependiendo del programa main a usar
void RBot::Init(bool view){
    Serial.begin(9600);
    InitRoboclaw(view);
    //InitIMU();

    roboclaw.SetM1MaxCurrent(Address,10*100); //max current 10A
    roboclaw.SetM2MaxCurrent(Address,10*100); //max current 10A

    _t1=millis();
    _t2=millis();
}

//ROBOCLAW                  //**********************************************//
void RBot::InitRoboclaw(bool view){
    //if(view) ViewConfValues();
    roboclaw.begin(38400);  
    //roboclaw.SetM2VelocityPID(Address,Kp1,Ki1,0,qpps1);
	//roboclaw.SetM2VelocityPID(Address,Kp2,Ki2,0,qpps2);
}

void RBot::ConfRoboclaw(uint8_t Address, int EncPPR, int Gear){
    this->Address = Address;//Dirección del Roboclaw
    this->EncPPR = EncPPR;  //Cuentas del encoder
    this->Gear = Gear;      //reducción de la caja
    
    this->QPPStoRPM = 60.0 / float((4.0*Gear*EncPPR));//Cte de conversión a RPM
}
void RBot::ConfMotor1(float Kp1, float Ki1, int qpps1){
    this->Kp1 = Kp1;    //Cte proporcional
    this->Ki1 = Ki1;    //Cte integral
    this->qpps1 = qpps1;//Cte quadratura(?)
    this->maxM1 = qpps1 * QPPStoRPM; //Vel. máx motor 1 en RPM
}
void RBot::ConfMotor2(float Kp2, float Ki2, int qpps2){
    this->Kp2 = Kp2;    //Cte proporcional
    this->Ki2 = Ki2;    //Cte integral
    this->qpps2 = qpps2;//Cte quadratura(?)
    this->maxM2 = qpps2 * QPPStoRPM; //Vel. máx motor 2 en RPM
}
void RBot::ViewConfValues(){
    int cant = 12, i = 0;
    String name[cant+1] = {"Address","EncPPR","Gear", "Kp1", "Ki1", "qpps1", "Kp2", "Ki2", "qpps2","maxM1","maxM2","QPPStoRPM"};
    String data[cant+1] = {String(Address), String(EncPPR), String(Gear), String(Kp1), String(Ki1), String(qpps1), String(Kp2), String(Ki2), String(qpps2), String(maxM1), String(maxM2), String(QPPStoRPM)};

    for(i = 0 ; i<cant; i++){
        Serial.print(name[i]); Serial.print(": ");
        Serial.println(data[i]);
    }
}

void RBot::SetSpeeds(float speed1, float speed2){
	float vel1 = speed1 / QPPStoRPM; //vel en QPPS, speed en RPM	
    float vel2 = speed2 / QPPStoRPM; //vel en QPPS, speed en RPM

    if(speed1 == 0) vel1 = 0;
    if(speed2 == 0) vel2 = 0;

	roboclaw.SpeedM1(Address,int(vel1));
    roboclaw.SpeedM2(Address,int(vel2));
    //Serial.println(int(vel1));
/*

    uint32_t c1, c2;
    bool xs;
    xs = roboclaw.ReadM1MaxCurrent(Address,c1);
    if(xs) Serial.print("Max1: "); Serial.print(c1);
    xs = roboclaw.ReadM2MaxCurrent(Address,c2);
    if(xs) Serial.print(" Max2: "); Serial.println(c2);
    
    int16_t current1, current2;
    roboclaw.ReadCurrents(Address, current1, current2);
    Serial.print("Amp1: "); Serial.print(current1/100);
    Serial.print("A Amp2: "); Serial.print(current2/100);
    Serial.println(" A");

*/
}

void RBot::GetSpeeds(){
  uint8_t status1,status2;
  bool valid1,valid2;
  
  int32_t speed1 = roboclaw.ReadSpeedM1(Address, &status1, &valid1); //en pulsos por segundo
  int32_t speed2 = roboclaw.ReadSpeedM2(Address, &status2, &valid2);

  if(valid1){
    Serial.print("V1: "); Serial.print(speed1); //velocidad en QPPS
    Serial.print("\t ");
    Serial.print(float(speed1) * QPPStoRPM); //velocidad en RPM
    Serial.print("\t ");

    if(millis() - _t1 > _tmax1){
      _tmax1 = millis() - _t1;
    }
        
    Serial.print("t1: "); Serial.print("\t ");
    Serial.print(_tmax1); Serial.print("\t ");

    _t1 = millis();
  } 
  else{
    Serial.print("V1: none"); Serial.print("\t ");
  }

  if(valid2){
    Serial.print("V2: "); Serial.print(speed2); //velocidad en QPPS
    Serial.print("\t ");
    Serial.print(float(speed2) * QPPStoRPM); //velocidad en RPM
    Serial.print("\t ");

    if(millis() - _t2 > _tmax2){
      _tmax2 = millis() - _t2;
    }

    Serial.print("t2: "); Serial.print("\t ");   
    Serial.println(_tmax2);
    _t2 = millis();
  } 
  else{
    Serial.print("V2: none"); Serial.println("\t ");
  }
}

void RBot::RobotControlSpeed(float veld, float thetad, float thetam){
   
    float thetae = thetad - thetam ;

    float W = K*(thetae); //en grados

    float wrSP = (veld/R + L*W/(2*R))/(6); //en RPM 180 1 rev-360, 60seg-1min, W=Veld*180/(R*pi), veld=veldp_p*180/pi, veld_p=veld*pi/180
    float wdSP = (veld/R - L*W/(2*R))/(6);

    /*if(wrSP>=343.0/6.0){wrSP=343.0/6.0;}
    if(wrSP<=-343.0/6.0){wrSP=-343.0/6.0;}*/

    if(wrSP>=425.0/6.0){wrSP=425.0/6.0;}
    if(wrSP<=-425.0/6.0){wrSP=-425.0/6.0;}

    if(wdSP>=425.0/6.0){wdSP=425.0/6.0;}
    if(wdSP<=-425.0/6.0){wdSP=-425.0/6.0;}

    SetSpeeds(wdSP,wrSP);

    /*Serial.println(thetae);
    Serial.println(wrSP);
    Serial.println(wdSP);*/
    }
    

void RBot::RobotControl(float xd, float xm, float thetad, float thetam, float vel){
        
    float xe = xd - xm;
    float thetae = thetad - thetam;

    float p1 = vel + K*xe;
    float p2 = K*thetae;

    float V = (1/cos(thetam)) * (p1 + sin(thetam)*p2);
    float W = (1/cos(thetam)) * cos(thetam)*p2;

    float wrSP = (V/R + L*W/(2*R))*180/(6*PI); //en RPM
    float wdSP = (V/R - L*W/(2*R))*180/(6*PI);

    SetSpeeds(wrSP,wdSP);
}


void RBot::LawControl(double angSP, double vSP, double angM){
    double angle = 0.0;
    angle = LimitValues(PI, (angSP - angM)*PI/180);

    double u1 = vSP*PI/180 + angle * KP;
    double u2 = vSP*PI/180 - angle * KP;

    double PWMu1 = ScaleValue(-24, 24, 0, 127, LimitValues(VoltLimit, u1));
    double PWMu2 = ScaleValue(-24, 24, 0, 127, LimitValues(VoltLimit, u2)); 

    //roboclaw.ForwardM1(Address,64); //start Motor1 forward at half speed
    //roboclaw.BackwardM2(Address,64); //start Motor2 backward at half speed
    //delay(2000);
    
    roboclaw.ForwardBackwardM1(Address,PWMu1);
    roboclaw.ForwardBackwardM1(Address,PWMu2);
    
    Serial.print("PWM1: "); Serial.print(PWMu1);
    Serial.print("\t PWM2: "); Serial.println(PWMu2);
}


//IMU                //**********************************************//
void RBot::InitIMU(){
    Wire.begin();
    Wire.setClock(400000);

    bool initialized = false;
    while (!initialized){
        myICM.begin(Wire, AD0_VAL);
    
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok){
          Serial.println("Trying again...");
          delay(500);
        }
        else{
          initialized = true;
        }
    }
}

void RBot::IMUData(ICM_20948_I2C *sensor){    
    if (myICM.dataReady()){
        myICM.getAGMT(); //update IMU data

        ax = sensor->accX(); //accelerometer (mg)
        ay = sensor->accY();
        az = sensor->accZ();
      
        gx = sensor->gyrX(); //gyroscope (DPS)
        gy = sensor->gyrY();
        gz = sensor->gyrZ();
      
        mx = sensor->magX(); //magnetometer (uT)
        my = sensor->magY();
        mz = sensor->magZ();
    }
}
void RBot::GetIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz){
    
    IMUData(&myICM);

    ax = this->ax;
    ay = this->ay;
    az = this->az;

    gx = this->gx;
    gy = this->gy;
    gz = this->gz;

    mx = this->mx;
    my = this->my;
    mz = this->mz;
}
float RBot::GetIMUAngle(){
    
    
    float angle = 2.0;

    return angle;
}
//función para obtener ángulos

//INCLINÓMETRO                //**********************************************//
void RBot::CompareIMUData(){

    //Serial.print((float)JY901.stcAngle.Angle[0]/32768*180); Serial.print("/");
    //Serial.print((float)JY901.stcAngle.Angle[1]/32768*180); Serial.print("/");
    //Serial.println((float)JY901.stcAngle.Angle[2]/32768*180);
    delay(50);

   

}




//PRINT                   //**********************************************//
void RBot::Print(float array[], int cant){
    
    for ( int i = 0 ; i < cant ; i++ ){  
        Serial.print(array[i]); 
        if(i<cant-1){
            Serial.print(", ");
        }
    } 
    Serial.println();
}
void RBot::Print(float array[], int cant, int decimals){
    
    for ( int i = 0 ; i < cant ; i++ ){  
        Serial.print(array[i], decimals); 
        if(i<cant-1){
            Serial.print(", ");
        }
    } 
    Serial.println();
}
void RBot::Print(float array[], int cant, int decimals, int view){

    if(_printCounter <= view){
        for ( int i = 0 ; i < cant ; i++ ){  
            Serial.print(array[i], decimals); 
            if(i<cant-1){
                Serial.print(", ");
            }
        } 
        if(view == 0){
            float _t1 = millis()/1000.0;
            Serial.print(", "); Serial.print(_t1, decimals); 
        }
        else{
            _printCounter++;
            Serial.print(", "); Serial.print(_printCounter, 0); 
        }
        Serial.println();
    }
}


//AUX                   //**********************************************//
double RBot::LimitValues(double range, double value){
    double val = value;

    if(value > range){
        value = range;
    }
    if(value < -range){
        value = -range;
    }

    return val;
}

double RBot::ScaleValue(double minX, double maxX, double minY, double maxY, double value){
    double m = (maxY - minY)/(maxX - minX);
    double b = minY - minX * m;

    int val = int(m*value + b);

    return val;
}

uint32_t RBot::ReturnSpeedM1(){//derecha
    uint8_t status1;
    uint32_t speedM1;
    bool valid1;

    speedM1 = roboclaw.ReadSpeedM1(Address, &status1, &valid1);
    if(valid1){
        return speedM1;
    }
    return 0;   
}

uint32_t RBot::ReturnSpeedM2(){
    
    uint8_t status2;
    uint32_t speedM2;
    bool valid2;
    speedM2 = roboclaw.ReadSpeedM2(Address, &status2, &valid2);
    if(valid2){
        return speedM2;
    }
    return 0;   
}