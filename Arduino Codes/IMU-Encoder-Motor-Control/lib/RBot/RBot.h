#ifndef RBOT_H
#define RBOT_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RoboClaw.h"
#include "ICM_20948.h"

//ROBOCLAW

//IMU
#define AD0_VAL 1      // The value of the last bit of the I2C address. 

//INCLINÓMETRO


class RBot /*: public RoboClaw*/ {
    private:
//Robot vars
        //float V = 0.0;
        //float W = 0.0;
        float R = 0.0165; //0.045;
        float L = 0.84;
        float K = 25;//10.0;

//Control vars
        float KP = 20.0;
        float VoltLimit = 20.0;
        float VMotor = 24.0;

//Roboclaw
        uint8_t Address = 0x80; //Dirección del Roboclaw
        int EncPPR = 600;     //Cuentas del encoder
        int Gear = 1;       //reducción de la caja
        float QPPStoRPM = 0.0;  //Cte de conversión
        float Kp1 = 0;        //Cte proporcional
        float Ki1 = 0;        //Cte integral
        int qpps1 = 0;      //Cte quadratura(?)
        float maxM1 = 0.0;      //Vel. máx motor 1 en RPM
        float Kp2 = 0;        //Cte proporcional
        float Ki2 = 0;        //Cte integral
        int qpps2 = 0;      //Cte quadratura(?)
        float maxM2 = 0.0;      //Vel. máx motor 2 en RPM
//IMU
        float ax = 0.0;
        float ay = 0.0;
        float az = 0.0;
        float gx = 0.0;
        float gy = 0.0;
        float gz = 0.0;
        float mx = 0.0;
        float my = 0.0;
        float mz = 0.0;
//VARs
        unsigned long _t1 = 0;
        unsigned long _t2 = 0;
        unsigned long _tmax1 = 0;
        unsigned long _tmax2 = 0;
        float _r = 0.0446;

//Print
        int _printCounter = 0;
    public:
//CONFIG (inicializar)
        void Init(bool view);
        
//**********************************************************************//
//ROBOCLAW
        void InitRoboclaw(bool view);
        void ConfRoboclaw(uint8_t Address, int EncPPR, int Gear);   
        void ConfMotor1(float kp1, float ki1, int qpps1);
        void ConfMotor2(float kp2, float ki2, int qpps2);
        void ViewConfValues();
        void SetSpeeds(float speed1, float speed2);
        void GetSpeeds();
        
        void RobotControl(float xd, float xm, float thetad, float thetam,  float xdp);
        void RobotControlSpeed(float veld, float thetad, float thetam);

        void LawControl(double angSP, double vSP, double angM);
    
//IMU
        void InitIMU();
        void IMUData(ICM_20948_I2C *sensor); 
        void GetIMUData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz);
        float GetIMUAngle();
 
//INCLINÓMETRO
        void CompareIMUData();
        
//PRINT: Imprimir en pantalla
        void Print(float array[], int cant);
        void Print(float array[], int cant, int decimals);
        void Print(float array[], int cant, int decimals, int view);
//AUX
        double LimitValues(double range, double value);
        double ScaleValue(double minX, double maxX, double minY, double maxY, double value);
        uint32_t ReturnSpeedM1();    // uint8_t *status1
        uint32_t ReturnSpeedM2();    // uint8_t *status2
};

#endif