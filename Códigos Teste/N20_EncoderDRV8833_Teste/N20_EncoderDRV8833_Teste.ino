#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"                     // TESTAR SEM
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <EnableInterrupt.h>

#define encoderLChA          13
#define encoderLChB          12
#define mR2                  11
#define mR1                  10
#define servoNeck            9
#define encoderRChA          8
#define encoderRChB          7
#define mL1                  6
#define mL2                  5
#define accGyrInt            2

// GYR/ACC
MPU6050 mpu(0x68);
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
float gravidade_mss = 9.80665;
int accFatorEscala = 4; // +- 4G

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float aceleracao[2][3];
float velocidade[2][3];
float posicao[2][3];
unsigned long tempoIntAccGyr[2];

bool chaveIntegrar = false;
//
bool chALState, chBLState, chALLastS, chBLLastS;
bool chARState, chBRState, chARLastS, chBRLastS;
long posL = 0, lastPosL = 0;
long posR = 0, lastPosR = 0;

float ppr = 2772; // Pulsos por rotação dos motores
float diametroRodas = 44; // Em mm
float raioRodas = diametroRodas / 2;
float entreRodas = 80;
float distPasso = (raioRodas * M_PI * (360 / ppr)) / 180;

float X = 0.0, Y = 0.0, Theta = 0.0;


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  
  acc_gyrSetup();

  pinMode(mL1, OUTPUT);
  pinMode(mL2, OUTPUT);
  pinMode(mR1, OUTPUT);
  pinMode(mR2, OUTPUT);
  pinMode(encoderLChA, INPUT_PULLUP);
  pinMode(encoderLChB, INPUT_PULLUP);
  pinMode(encoderRChA, INPUT_PULLUP);
  pinMode(encoderRChB, INPUT_PULLUP);
  chALLastS = digitalRead(encoderLChA);
  chBLLastS = digitalRead(encoderLChB);

//  enableInterrupt(encoderLChA, handleLEncoder, CHANGE);
//  enableInterrupt(encoderLChB, handleLEncoder, CHANGE);
//  enableInterrupt(encoderRChA, handleREncoder, CHANGE);
//  enableInterrupt(encoderRChB, handleREncoder, CHANGE);

  Serial.println("Starting...");
}

void loop() {

  while (Serial.available() > 0) bluetoothControl();
  //  checkPos();

  receberAccGyr();
}
