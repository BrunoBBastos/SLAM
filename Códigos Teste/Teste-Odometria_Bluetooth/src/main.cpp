#include "Arduino.h"
#include "MsTimer2.h"

#define ENC_LA               13
#define ENC_LB               12
#define MRB                  10
#define MRA                  11
#define SERVO                9
#define ENC_RA               8
#define ENC_RB               7
#define MLA                  5
#define MLB                  6

////////////////////////////////////////////////////// PARÂMETROS
float roda_raio = 22.0/1000;
float l = 120.0/1000;
float dPhi = 2*PI / 2091.0;
float dt = 20.0/1000;
volatile long pulsosR = 0, pulsosL = 0;
float x = 0.0, y = 0.0, theta = 0.0;

////////////////////////////////////////////////////// Testes
float k1 = 0.850, k2 = 0.20;

////////////////////////////////////////////////////// Protótipos

void PCISetup(byte pin);
void controleManual(int esquerdo, int direito);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
void odometria();
float angleWrap(float ang);
void bluetoothControl();
void printOdom();
void mover(float Xref, float Yref);
void controleRef(float Xref, float Yref, float ctrl[2]);


////////////////////////////////////////////////////// SETUP

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(ENC_RB, INPUT_PULLUP);
  pinMode(ENC_RA, INPUT_PULLUP);
  pinMode(MLA, OUTPUT);
  pinMode(MLB, OUTPUT);
  pinMode(MRA, OUTPUT);
  pinMode(MRB, OUTPUT);

  PCISetup(ENC_LB);
  PCISetup(ENC_RB);

// Colocar dentro do menu 'modo de operação'
  Serial.println("Envie qqr coisa para acionar");
  while(Serial.available() == 0);
  Serial.print("Acionando motor\nEnvie qqr coisa para parar \n");
  while(Serial.available() != 0) Serial.read();

// PRA FRENTE:
//  digitalWrite(MLA, 1);
//  digitalWrite(MLB, 0);
//  digitalWrite(MRA, 0);
//  digitalWrite(MRB, 1);

  MsTimer2::set(dt*1000, odometria);
  MsTimer2::start();
}

////////////////////////////////////////////////////// LOOP

void loop() {
  // put your main code here, to run repeatedly:
  mover(100, 0);
  if(Serial.available() > 0)
  {
    bluetoothControl();
  }
}

////////////////////////////////////////////////////// FUNÇÕES

void PCISetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}


void controleManual(int esquerdo, int direito)
{
 if (esquerdo == 1) {
    digitalWrite(MLA, 1);
    digitalWrite(MLB, 0);
  }
  else if (esquerdo == 0) {
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 0);
  }
  else if (esquerdo == -1) {
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 1);
  }
  if (direito == 1) {
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 1);
  }
  else if (direito == 0) {
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 0);
  }
  else if (direito == -1) {
    digitalWrite(MRA, 1);
    digitalWrite(MRB, 0);
  }
}

ISR (PCINT0_vect) // Interrupt Service Routine dos pinos 8 a 13
{
  bool A = digitalRead(ENC_LA);
  bool B = digitalRead(ENC_LB);

  if(A == B)
  {
    pulsosL--;
  }
  else
  {
    pulsosL++;
  }
}

ISR (PCINT2_vect) // Interrupt Service Routine dos pinos 0 a 7
{
  bool A = digitalRead(ENC_RA);
  bool B = digitalRead(ENC_RB);

  if(A == B)
  {
    pulsosR++;
  }
  else
  {
    pulsosR--;
  }
}

void odometria()
{
  // Velocidade linear da roda: v = w*r
  float Vr = pulsosR * (dPhi / dt) * roda_raio;
  float Vl = pulsosL * (dPhi / dt) * roda_raio;
  pulsosL = 0;
  pulsosR = 0;
  
  if (Vr == Vl) // Se o robô estiver andando em linha reta:
  {
    x += Vr * cos(theta) * dt;
    y += Vr * sin(theta) * dt;
    return;
  }
  
  // Velocidade angular do robô: (Vr - Vl)/l
  float w = (Vr - Vl) / l;
  // Distância para o centro de curvatura:
  float R = (l / 2) * (Vr + Vl)/(Vr - Vl);
  // Posição do Centro de Curvatura Instantâneo:
  float CCIx = x - R * sin(theta);
  float CCIy = y + R * cos(theta);

  x = cos(dt*w) * (x - CCIx) - sin(dt*w) * (y - CCIy) + CCIx;
  y = sin(dt*w) * (x - CCIx) + cos(dt*w) * (y - CCIy) + CCIy;
  theta += dt*w;
  theta = angleWrap(theta);
}

float angleWrap(float ang)
{
  if (ang > PI)
  {
    return ang - 2 * PI;
  }
  else if (ang < -PI)
  {
    return ang + 2 * PI;
  }
  else
  {
    return ang;
  }
}


void bluetoothControl()
{
  char buff = Serial.read();

  switch (buff) {

    case 'F':
      controleManual(1, 1);
      break;

    case 'B':
      controleManual(-1, -1);
      break;

    case 'L':
      controleManual(-1, 1);
      break;

    case 'R':
      controleManual(1, -1);
      break;

    case 'G':
      controleManual(0, 1);
      break;

    case 'I':
      controleManual(1, 0);
      break;

    case 'H':
      controleManual(0, -1);
      break;

    case 'J':
      controleManual(-1, 0);
      break;

    case 'S':
      controleManual(0, 0);
      break;

    case 'x':
      printOdom();
      break;
  }
}

void printOdom()
{
  Serial.println(x);
  Serial.println(y);
  Serial.println(theta);
  Serial.println();
}

void controleRef(float Xref, float Yref, float ctrl[2])
{
  float dY = (Yref - y);
  float dX = (Xref - x);
  float THETAref = atan2(dY, dX);
  THETAref = angleWrap(THETAref);

  float erroAngular = THETAref - theta;
  float erroLinear = sqrt(pow(dX, 2) + pow(dY, 2)) * cos(erroAngular);

  float v = k1 * erroLinear;
  float w = k2 * erroAngular;
  // Serial.print(erroAngular);
  // Serial.print(" ");
  // Serial.println(erroLinear);
  ctrl [0] = v + w;
  ctrl [1] = v - w;
}

void mover(float Xref, float Yref)
{
  if (sqrt(pow(Xref - x, 2) + pow(Yref- y, 2)) < 50)
  {
    Serial.println("Chegou");
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 0);
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 0);
    return;
  }

  float ctrl[2];
  controleRef(Xref, Yref, ctrl);
  ctrl[0] = int(constrain(ctrl[0], -255, 255));
  ctrl[1] = int(constrain(ctrl[1], -255, 255));

  // Serial.print(ctrl[0]);
  // Serial.print(' ');
  // Serial.println(ctrl[1]);

  if (ctrl[0] > 0)
  {
    analogWrite(MLA, abs(ctrl[0]));
    digitalWrite(MLB, 0);
  }
  else
  {
    digitalWrite(MLA, 0);
    analogWrite(MLB,  abs(ctrl[0]));
  }

  if (ctrl[1] > 0)
  {
    digitalWrite(MRA, 0);
    analogWrite(MRB, abs(ctrl[1]));
  }
  else
  {
    analogWrite(MRA, abs(ctrl[1]));
    digitalWrite(MRB,  0);
  }

}