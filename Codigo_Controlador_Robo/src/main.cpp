////////////////////////////////////////////////////// BIBLIOTECAS

#include "Arduino.h"

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
const int tDeltaMillis = 20;
const double dt = tDeltaMillis/1000.0;
const double dPhi = 2*PI / 2091.0;
const double RPM = 100.0;

double Pose[3] = {0.0, 0.0, 0.0}; 
volatile long int pulsosEncL = 0, pulsosEncR = 0;



void PCISetup(byte pin);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
float angleWrap(float ang);
double distancia2D(float p1[2], float p2[2]);
void odometria();
void ouvirSerial();
void velocidadeCombinarComponentes(float linear, float angular, float vw[2]);
int convertePWM(float sinal);
void acionarMotores(int pwmE, int pwmD);
void acionar(int pwm, int MA, int MB);
float angleWrap(float ang);


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

void ouvirSerial()
{
  int cmd;
  if (Serial.available() > 0)
  {
    cmd = Serial.read();
    Serial.println(cmd);

    switch(cmd)
    {
      // Receber componente linear e angular de velocidade das rodas
      case 'V':
      {
        float v = Serial.parseFloat(SKIP_WHITESPACE);
        float w = Serial.parseFloat(SKIP_WHITESPACE);
        float ctrl[2];
        

        velocidadeCombinarComponentes(v, w, ctrl);

        ctrl[0] = convertePWM(ctrl[0]);
        ctrl[1] = convertePWM(ctrl[1]);
        acionarMotores(ctrl[0], ctrl[1]);
      }
      break;

      // Informar o último incremento de posição
      case 'O':
      {
        Serial.print(Pose[0]);
        Serial.print(', ');
        Serial.print(Pose[1]);
        Serial.print(', ');
        Serial.println(Pose[2]);
        
        noInterrupts();
        Pose[0] = 0.0;
        Pose[1] = 0.0;
        Pose[2] = 0.0;
        interrupts();
      }
      break;
    }
  }
}

void velocidadeCombinarComponentes(float linear, float angular, float vw[2])
{
  vw[0] = linear - angular;
  vw[1] = linear + angular;
}

int convertePWM(float sinal)
{
  float maxVel = (RPM/60) * 2 * PI * roda_raio;
  int pwm = sinal * 255 / maxVel;
  int sig = abs(pwm)/ pwm;
  pwm = constrain(abs(pwm), 0, 255) * sig;
  return pwm;
}

void acionarMotores(int pwmE, int pwmD)
{
  acionar(pwmE, MLA, MLB);
  acionar(pwmD, MRB, MRA);
}

void acionar(int pwm, int MA, int MB)
{
  if (pwm > 0)
  {
    analogWrite(MA, pwm);
    digitalWrite(MB, 0);
  }
  else{
    analogWrite(MA, 0);
    digitalWrite(MB, pwm);  
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

double distancia2D(float p1[2], float p2[2])
{
  return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
}

//##############################################################################
//##############################################################################
//##############################################################################
//##############################################################################


unsigned long long int tLast, tCurr;

void setup()
{
    Serial.begin(115200);

    PCISetup(ENC_LB);
    PCISetup(ENC_RB);

    tCurr = millis();
    tLast = tCurr;
}

void loop()
{
  tCurr = millis() - tLast;
  if (tCurr >= tDeltaMillis)
  {
    odometria();
  }

  ouvirSerial();

}

