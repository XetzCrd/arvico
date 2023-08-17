#include <Arduino.h>
#include <RBot.h>
#include <Wire.h>
#include <JY901.h>
#include <ModbusMasterFP.h>
#include <SoftwareSerial.h>
#include "ICM_20948.h"
#include <Kalman.h> 


RBot rb;
ModbusMasterFP node;
SoftwareSerial mySerial(51,53);//28,29 RX, TX //WTC901C
ICM_20948_I2C IMURojo;           // Otherwise create an ICM_20948_I2C object
Kalman kalmanY;

//Velocity PID coefficients.
#define Kp1 1.07993 //2.11730
#define Ki1 0.11999 //0.75183
#define qpps1 22500 //2625

#define Kp2 1.11107 //2.75726
#define Ki2 0.11984 //0.93567
#define qpps2 22875 //2625

#define MAX485_DE      28//5//3 //Driver output Enable pin DE Active HIGH
#define MAX485_RE_NEG  26//4//2 //receiver output Enable pin RE Active LOW

//Encoder
#define ENCODER_A 2
#define ENCODER_B 22
volatile int pulsos = 0;
float resolution = 600;          // PPR
float wheel_circunference = 200; // 200 mm //alvaro 150 mm

// Variables globales
char incomingByte = 0;
uint8_t estado = 0;
bool inclinometro = true;
bool nuevaOpcion = true;
float vel = 11.45; // -0.1
float set_point = 0;
uint8_t result;
double angle;
float pos_deseada = 0.2; // 1 m
float pulsos_req = (pos_deseada * 1000.0) * (1.0 / wheel_circunference) * (resolution);
int inicio = 0;
int grabar = 0;
uint32_t k = 0;
const uint32_t PerMuestras = 40; // 5 ms // 10 ms

// Variables IMU Rojo
double ax, ay, az, gy;
double axP, ayP, azP, gyP; // Past values
double kalAngleY;          // Calculated angle using a Kalman filter
unsigned long timer = 0;

// Dirección del Roboclaw //const uint8_t Address = 0x80;

// Function leerEncoder: Modificar si se busca un giro horario o antihorario
void leerEncoder()
{
  if (digitalRead(ENCODER_B) == LOW)
  { // Horario
    pulsos++;
  }
  else
  { // Antihorario
    pulsos--;
  }
}
// Funcion TomaLecturas: Registra los valores del ángulo roll, pulsos y velocidades de los motores 1 y 2 en arreglos
bool TomaLecturas(uint16_t *p_NumLecturas = nullptr, uint16_t **p_RegistraAngulos = nullptr, int **p_RegistraPulsos = nullptr, uint32_t **p_RegistraVelocidadM1 = nullptr, uint32_t **p_RegistraVelocidadM2 = nullptr, uint16_t **p_RegistraAngulos2Inc = nullptr)
{
  static const uint16_t NUM_LECTURAS = 100; // 350
  static uint16_t i = 0;
  static uint16_t AngulosReg[NUM_LECTURAS];
  static int PulsosReg[NUM_LECTURAS];
  static uint32_t VelRegM1[NUM_LECTURAS];
  static uint32_t VelRegM2[NUM_LECTURAS];
  static uint16_t Angulos2Inc[NUM_LECTURAS];
  static uint32_t t_ini = millis();
  bool bufferLleno = false;
  uint32_t t_now = millis();

  if (t_now - t_ini >= PerMuestras)
  {
    t_ini += PerMuestras;
    if (inicio == 1)
    {
      if (inclinometro)
      {
        AngulosReg[i] = node.getResponseBuffer(0x00); // Ángulo roll Industrial
      }
      else
      {
        AngulosReg[i] = 0; // Ángulo roll Rojo
      }

      PulsosReg[i] = pulsos;                    // Registro de pulsos
      VelRegM1[i] = rb.ReturnSpeedM1();         // Registro de velocidades M1
      VelRegM2[i] = rb.ReturnSpeedM2();         // Registro de velocidades M2 y status 2
      Angulos2Inc[i] = JY901.stcAngle.Angle[1]; // Registro de angulos del 2do Inclinometro

      i++;
      if (i >= NUM_LECTURAS)
      {
        bufferLleno = true;
        i = 0;
      }
    }
  }
  if (p_NumLecturas != nullptr)
  {
    *p_NumLecturas = NUM_LECTURAS;
  }
  if (p_RegistraAngulos != nullptr)
  {
    *p_RegistraAngulos = AngulosReg;
  }
  if (p_RegistraPulsos != nullptr)
  {
    *p_RegistraPulsos = PulsosReg;
  }
  if (p_RegistraVelocidadM1 != nullptr)
  {
    *p_RegistraVelocidadM1 = VelRegM1;
  }
  if (p_RegistraVelocidadM2 != nullptr)
  {
    *p_RegistraVelocidadM2 = VelRegM2;
  }
  if (p_RegistraAngulos2Inc != nullptr)
  {
    *p_RegistraAngulos2Inc = Angulos2Inc;
  }
  return bufferLleno;
}
void preTransmission1() // set up call back function
{
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}
void postTransmission1() // set up call back function
{
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}
void moverMotores(int m1, int m2) // Mueve cada motor a una velocidad dada
{
  // Serial.println("Inicio");
  Serial.print(">> M1 ");
  Serial.print(m1);
  Serial.print(" RPM y M2 ");
  Serial.print(m2);
  Serial.println(" RPM");

  rb.SetSpeeds(m1, m2); // Positivo sube
  delay(3000);
  Serial.print("Fin\n");
  rb.SetSpeeds(0, 0);
  delay(2000);
}
void IMUData(ICM_20948_I2C *sensor)
{
  float actLPF = 0.84816; // filtro de 30 Hz
  float antLPF = 0.15184;

  if (IMURojo.dataReady())
  {
    IMURojo.getAGMT(); // update IMU data

    ax = sensor->accX() - 0.007; // accelerometer (mg)
    ay = sensor->accY() - 0.002;
    az = sensor->accZ() - 0.006;

    gy = sensor->gyrY() + 1.129;
  }

  ax = antLPF * axP + actLPF * ax;
  ay = antLPF * ayP + actLPF * ay;
  az = antLPF * azP + actLPF * az;
  gy = antLPF * gyP + actLPF * gy;

  axP = ax;
  ayP = ay;
  azP = az;
  gyP = gy;
}
void updateK_Angle(Kalman *kObject, double *k_angle){
  IMUData(&IMURojo);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double kvalue = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG; //ROLL Y axis


  if ((kvalue < -90 && *k_angle > 90) || (kvalue > 90 && *k_angle < -90))
  {
    kObject->setAngle(kvalue);
    *k_angle = kvalue;
  }
  else
  {
    *k_angle = kObject->getAngle(kvalue, gy, dt); // Calculate the angle using a Kalman filter
  }
  
}

void menu()
{
  Serial.println("\nMENU DE OPCIONES:");
  Serial.println("q: Subir los 2 motores");
  Serial.println("a: Bajar los 2 motores");
  Serial.println("w: Subir motor izquierdo");
  Serial.println("s: Bajar motor izquierdo");
  Serial.println("e: Subir motor derecho");
  Serial.println("d: Bajar motor derecho");
  Serial.println("#");
  Serial.println("z: Mostrar angulos de libertad");
  Serial.println("y: Control de desplazamiento: ");
  Serial.println("#");
  Serial.println("i: Mostrar menú de opciones");
}
/*
void IMUData(ICM_20948_I2C *sensor) // Actualiza los datos del IMU Rojo
{
  float actLPF = 0.95679;
  float antLPF = 0.04321;

  if (IMURojo.dataReady())
  {
    IMURojo.getAGMT(); // update IMU data

    ax = sensor->accX() - 0.007; // accelerometer (mg)
    ay = sensor->accY() - 0.002;
    az = sensor->accZ() - 0.006;

    gy = sensor->gyrY() + 1.129; // gyroscope (dps)
  }

  ax = antLPF * axP + actLPF * ax;
  ay = antLPF * ayP + actLPF * ay;
  az = antLPF * azP + actLPF * az;
  gy = antLPF * gyP + actLPF * gy;

  axP = ax;
  ayP = ay;
  azP = az;
  gyP = gy;
}
void updateK_Angle(Kalman *kObject, double *k_angle) // Update angle IMU Rojo
{ 
  IMUData(&IMURojo);

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double kvalue = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG; // ROLL Y axis

  if ((kvalue < -90 && *k_angle > 90) || (kvalue > 90 && *k_angle < -90))
  {
    kObject->setAngle(kvalue);
    *k_angle = kvalue;
  }
  else
  {
    *k_angle = kObject->getAngle(kvalue, gy, dt); // Calculate the angle using a Kalman filter
  }
}
*/
void setup()
{
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), leerEncoder, RISING);

  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  // Init in receive mode
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
  // Modbus communication runs at 9600 baud
  Serial1.begin(9600); //serial 1: RX1 and TX1 in Arduino Mega
  // Modbus slave ID 1, numbers are in decimal format
  node.begin(80, Serial1);  //data from max 485 are communicating with serial1

  Serial.begin(9600);
  mySerial.begin(9600); // inicio WTC901C

/*

  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {
    IMURojo.begin(Wire, 1);

    Serial.println(IMURojo.statusString());
    if (IMURojo.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  delay(1000);


*/
  rb.ConfRoboclaw(0x80, 600, 1); // 600 1
  rb.ConfMotor1(Kp1, Ki1, qpps1);
  rb.ConfMotor2(Kp2, Ki2, qpps2);
  rb.Init(1);

  /*
  // Inicializa IMU Rojo
  Wire.begin();
  Wire.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {
    IMURojo.begin(Wire, 1);

    Serial.println(IMURojo.statusString());
    if (IMURojo.status != ICM_20948_Stat_Ok)
    {
      Serial.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  delay(1000);
  IMUData(&IMURojo);
  double roll = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  kalmanY.setAngle(roll);
  delay(100);
  timer = micros();
  */ 
  IMUData(&IMURojo);
  double roll = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
  kalmanY.setAngle(roll);
  delay(100);

  timer = micros();

  menu();
}

void loop()
{
  uint16_t NumLecturas;
  uint16_t *RegistraAngulos;
  int *RegistraPulsos;
  uint32_t *RegistraVelocidadM1; // Pulsos por segundo
  uint32_t *RegistraVelocidadM2; // Pulsos por segundo
  uint16_t *RegistraAngulos2Inc;
  bool FinLectura = TomaLecturas(&NumLecturas, &RegistraAngulos, &RegistraPulsos, &RegistraVelocidadM1, &RegistraVelocidadM2, &RegistraAngulos2Inc);

  if (FinLectura) // Muestra los datos de roll, pulsos, Vel M1 y Vel M2
  {
    inicio = 0;
    while (k < NumLecturas)
    {
      if (k == 0)
      {
        Serial.print("\nN°\t");
        Serial.print("roll\t");
        Serial.print("pulsos\t");
        Serial.print("Vel_M1\t");
        Serial.print("Vel_M2\t");
        Serial.println("WT901C\t");
      }
      Serial.print(k + 1);
      Serial.print("\t");
      Serial.print(RegistraAngulos[k]);
      Serial.print("\t");
      Serial.print(RegistraPulsos[k]);
      Serial.print("\t");
      Serial.print(RegistraVelocidadM1[k]);
      Serial.print("\t");
      Serial.print(RegistraVelocidadM2[k]);
      Serial.print("\t");
      Serial.println(RegistraAngulos2Inc[k]);

      k++;
    }
    k = 0;
    FinLectura = 0;
  }

  if (mySerial.available() > 0) // Actualizar ángulo del WT901C
  {
    JY901.CopeSerialData(mySerial.read());
  }

  if (estado == 0) // Funcionalidades del robot
  {
    if (nuevaOpcion && inicio == 0) // Mensaje para ingreso de opción
    {
      Serial.print("\n>> Ingrese opción: ");
      nuevaOpcion = false;
    }

    if (Serial.available() > 0) // read the incoming byte
    {
      incomingByte = Serial.read();
      Serial.println(incomingByte);
      Serial.println();
      nuevaOpcion = true;
    }

    if (incomingByte == 'q') // Suben los 2 motores
    {
      incomingByte = '_';
      Serial.println("Subiendo robot");
      moverMotores(20, 20);
    }
    else if (incomingByte == 'a') // Bajan los 2 motores
    {
      incomingByte = '_';
      Serial.println("Bajando robot");
      moverMotores(-20, -20);
    }
    else if (incomingByte == 'w') // Sube motor izquierdo
    {
      incomingByte = '_';
      Serial.println("Subiendo motor izquierdo");
      moverMotores(20, 0);
    }
    else if (incomingByte == 's') // Baja motor izquierdo
    {
      incomingByte = '_';
      Serial.println("Bajando motor izquierdo");
      moverMotores(-20, 0);
    }
    else if (incomingByte == 'e') // Sube motor derecho
    {
      incomingByte = '_';
      Serial.println("Subiendo motor derecho");
      moverMotores(0, 20);
    }
    else if (incomingByte == 'd') // Baja motor derecho
    {
      incomingByte = '_';
      Serial.println("Bajando motor derecho");
      moverMotores(0, -20);
    }
    else if (incomingByte == 'z') // Muestra ángulos de libertad
    {
      incomingByte = '_';

      Serial.println("========= \t MILITAR \t =========");
      result = node.readHoldingRegisters(0x3d, 3);
      if (result == node.ku8MBSuccess)
      {
        Serial.print("roll: ");
        angle = (double)node.getResponseBuffer(0x00) * 180.0 / 32768.0;
        if (angle >= 180)
        {
          angle = (-angle + 360);
        }
        else
        {
          angle = -angle;
        }
        Serial.println(angle);
/*
        Serial.print("\t pitch: ");
        angle = (double)node.getResponseBuffer(0x01) * 180.0 / 32768.0;
        Serial.print(angle);
        Serial.print("\t yaw: ");
        Serial.println(node.getResponseBuffer(0x02) * 180.0 / 32768.0);
*/        
      }
      else
      {
        Serial.println(result);
      }
      
      Serial.println("========= \t ROJO \t\t =========");
      updateK_Angle(&kalmanY, &kalAngleY);

      Serial.print("roll: ");
      Serial.println(kalAngleY);
      
    }
    else if (incomingByte == 'y') // Control de desplazamiento
    {
      incomingByte = '_';
      Serial.print("Ingrese la posición [+]: ");
      while (Serial.available() == 0)
      {
      }
      pos_deseada = Serial.parseFloat();
      Serial.println(pos_deseada);
      pulsos_req = (pos_deseada * 1000.0) * (1.0 / wheel_circunference) * (resolution);

      Serial.print("Ingrese la velocidad [+]Subir [-]Bajar: ");
      while (Serial.available() == 0)
      {
      }
      vel = Serial.parseFloat();
      Serial.println(vel);

      inclinometro = true;
      Serial.print("Escoja inclinómetro [0]Rojo [1]Industrial: ");
      while (Serial.available() == 0)
      {
      }
      int val_inc = Serial.parseInt();
      if (val_inc == 0)
      {
        inclinometro = false;
      }
      Serial.println(val_inc);

      inicio = 1;
      estado = 1;
      pulsos = 0;
      Serial.println("Fin configuración inicial");
      Serial.println("Inicia desplazamiento");
    }
    else if (incomingByte == 'i') // Muestra menú de opciones
    {
      incomingByte = '_';
      menu();
    }
  }
  else // Estado Control de Desplazamiento
  {
    result = node.readHoldingRegisters(0x3d, 3);
    if (result == node.ku8MBSuccess)
    {
      angle = (double)node.getResponseBuffer(0x00) * 180.0 / 32768.0;
    }
    else
    {
      Serial.println(result);
    }

    if (angle >= 180)
    {
      angle = (-angle + 360);
    }
    else
    {
      angle = -angle;
    }

    if (abs(pulsos) < pulsos_req)
    {
      rb.RobotControlSpeed(vel, set_point, angle);
    }
    else
    {
      rb.SetSpeeds(0, 0);
      Serial.println("Fin desplazamiento");
      pulsos = 0;
      estado = 0;
    }
  }
}
