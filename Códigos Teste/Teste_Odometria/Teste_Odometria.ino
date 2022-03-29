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
long tPrev;


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


  Serial.println("Envie qqr coisa para acionar");
  while(Serial.available() == 0);
  Serial.print("Acionando motor\nEnvie qqr coisa para parar \n");
  while(Serial.available() != 0) Serial.read();

  digitalWrite(MLA, 1);
  digitalWrite(MLB, 0);
  digitalWrite(MRA, 0);
  digitalWrite(MRB, 1);

  MsTimer2::set(dt*1000, odometria);
  MsTimer2::start();
//  tPrev = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)
  {
//    bluetoothControl();
  }

//  long t = millis();
//  if (t - tPrev >= dt*1000)
//  {
//    odometria();
//    tPrev = t;
//  }
}

////////////////////////////////////////////////////// FUNÇÕES

void PCISetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
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

//void bluetoothControl() {
//  char buff = Serial.read();
//
//  switch (buff) {
//
//    case 'F':
//      direcao(1, 1);
//      break;
//
//    case 'B':
//      direcao(-1, -1);
//      break;
//
//    case 'L':
//      direcao(-1, 1);
//      break;
//
//    case 'R':
//      direcao(1, -1);
//      break;
//
//    case 'G':
//      direcao(0, 1);
//      break;
//
//    case 'I':
//      direcao(1, 0);
//      break;
//
//    case 'H':
//      direcao(0, -1);
//      break;
//
//    case 'J':
//      direcao(-1, 0);
//      break;
//
//    case 'S':
//      direcao(0, 0);
//      break;
//
//    case 'x':
//      printOdom();
//      break;
//  }
//}

void printOdom()
{
  Serial.println(x);
  Serial.println(y);
  Serial.println(theta);
  Serial.println();
}
