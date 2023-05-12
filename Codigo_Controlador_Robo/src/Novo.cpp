////////////////////////////////////////////////////// BIBLIOTECAS

#include "Arduino.h"
#include "MsTimer2.h"

////////////////////////////////////////////////////// DEFINES

#define ENC_LA               13
#define ENC_LB               12
#define MRB                  10
#define MRA                  11
#define SERVO                9
#define ENC_RA               8
#define ENC_RB               7
#define MLA                  5
#define MLB                  6

const double roda_raio = 0;
const double l = 120.0/1000;
const double dt = 20.0/1000;
const double dPhi = 2*PI / 2091.0;
float Pose[3] = {0.0, 0.0, 0.0}; 
volatile long int pulsosEncL = 0, pulsosEncR = 0;

void PCISetup(byte pin);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
float angleWrap(float ang);
float distancia2D(float p1[2], float p2[2]);

//CONTROLE
float kpLinear = 0.5, kpAngular = 0.2;
float ref[3];

void gerarSinalControlePosicao(float pose[3], float referencia[3], float ctrl[2]);


// MOTORES
class Motor
{

  int A, B;

  public:
  void begin(int pinA, int pinB)
  {
    A = pinA;
    B = pinB;
    pinMode(A, OUTPUT);
    pinMode(B, OUTPUT);
  }

  void acionar(int pwm)
  {
    if (pwm > 0)
    {
      analogWrite(A, pwm);
      digitalWrite(B, 0);
    }
    else{
      analogWrite(A, 0);
      digitalWrite(B, pwm);  
    }
  }
};

class Encoder
{

  int A, B;
  volatile long int *pulsos;
  float dPhi;
  float dt = 20.0f/1000;

  public:
  void begin(int pinA, int pinB, float deltaPhi, volatile long int &varPulsos)
  {
    A = pinA;
    B = pinB;
    dPhi = deltaPhi;
    pulsos = &varPulsos;
    pinMode(A, INPUT_PULLUP);
    pinMode(B, INPUT_PULLUP);
  }

  // Observa a contagem de pulsos sem resetar o valor
  long int contar()
  {
    return *pulsos;
  }

  // Observa a contagem de pulsos e reseta o valor
  long int coletarPulsos()
  {
    long int total = *pulsos;
    *pulsos = 0;
    return total;
  }

  float velocidadeAngRoda()
  {
    return coletarPulsos() * dPhi/dt;
  }

};

Motor MEsq, MDir;
Encoder EEsq, EDir;
bool flagEnc;

void setup()
{
    Serial.begin(115200);

    PCISetup(ENC_LB);
    PCISetup(ENC_RB);
}


void loop()
{

}


void gerarSinalControlePosicao(float pose[3], float referencia[3], float ctrl[2])
{
  float dY = (referencia[1] - pose[1]);
  float dX = (referencia[0] - pose[0]);
  float THETAref = atan2(dY, dX);
  THETAref = angleWrap(THETAref);

  float erroAngular = angleWrap(THETAref - pose[2]);
  float erroLinear = sqrt(pow(dX, 2) + pow(dY, 2)) * cos(erroAngular);

  float v = kpLinear * erroLinear;
  float w = kpAngular * erroAngular;

  ctrl[0] = v - w;
  ctrl[1] = v + w;
}


void odometria()
{
    float vE = pulsosEncL * dPhi/dt * roda_raio;
    float vD = pulsosEncR * dPhi/dt * roda_raio;
    pulsosEncL = 0;
    pulsosEncR = 0;

    if (vE == vD)
    {
        Pose[0] += vD * cos(Pose[2]) * dt; 
        Pose[1] += vD * sin(Pose[2]) * dt;
    }

    else
    {
        float w = (vD - vE) / l;
        float R = (l/2) * (vD + vE) / (vD - vE);

        float CCIx = Pose[0] - R * sin(Pose[2]);
        float CCIy = Pose[1] + R * cos(Pose[2]);

        Pose[0] = cos(dt * w) * (Pose[0] - CCIx) - sin(dt*w) * (Pose[1] - CCIy) + CCIx;
        Pose[1] = sin(dt * w) * (Pose[0] - CCIx) + cos(dt*w) * (Pose[1] - CCIy) + CCIy;
        Pose[2] = Pose[2] + dt*w;
        Pose[2] = angleWrap(Pose[2]);
    }
}


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
    pulsosEncL--;
  }
  else
  {
    pulsosEncL++;
  }
}

ISR (PCINT2_vect) // Interrupt Service Routine dos pinos 0 a 7
{
  bool A = digitalRead(ENC_RA);
  bool B = digitalRead(ENC_RB);

  if(A == B)
  {
    pulsosEncR++;
  }
  else
  {
    pulsosEncR--;
  }
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
  return ang;
}

float distancia2D(float p1[2], float p2[2])
{
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
}