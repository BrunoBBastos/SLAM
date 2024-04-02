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

#define DEBUG 1

////////////////////////////////////////////////////////////////
//Variáveis de controle
float kp = 1, kd = 1;
const double roda_raio = 31.75/1000;
const double dPhi = 2*PI / 2091.0;

unsigned long odomInterval = 25, tLastOdom = 0;

const double RPM = 100.0;
const double maxVel = (RPM/60.0) * 2 * PI * roda_raio;

volatile long int pulsosEncL = 0, pulsosEncR = 0;

float velLeftSetpoint = 0.2, velRightSetpoint = 0.0;

//////////////////////////////////////////////////////////////////
//Protótipos
void PCISetup(byte pin);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
float angleWrap(float ang);
double distancia2D(float p1[2], float p2[2]);
void odometria();
void ouvirSerial();
int convertePWM(float sinal);
void acionarMotores(int pwmE, int pwmD);
void acionar(int pwm, int MA, int MB);
float angleWrap(float ang);


// void odometria()
// {
//   float vE = pulsosEncL * dPhi/dt * roda_raio;
//   float vD = pulsosEncR * dPhi/dt * roda_raio;
//   pulsosEncL = 0;
//   pulsosEncR = 0;

//   if (vE == vD)
//   {
//       Pose[0] += vD * cos(Pose[2]) * dt; 
//       Pose[1] += vD * sin(Pose[2]) * dt;
//   }

//   else
//   {
//       float w = (vD - vE) / l;
//       float R = (l/2) * (vD + vE) / (vD - vE);

//       float CCIx = Pose[0] - R * sin(Pose[2]);
//       float CCIy = Pose[1] + R * cos(Pose[2]);

//       Pose[0] = cos(dt * w) * (Pose[0] - CCIx) - sin(dt*w) * (Pose[1] - CCIy) + CCIx;
//       Pose[1] = sin(dt * w) * (Pose[0] - CCIx) + cos(dt*w) * (Pose[1] - CCIy) + CCIy;
//       Pose[2] = angleWrap(Pose[2] + dt*w);
//   }
// }

void ouvirSerial()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();

    if (cmd == 'V')
    {
      float l = Serial.parseFloat();
      float r = Serial.parseFloat();

      if (!isnan(l) && !isnan(r))
      {
        velLeftSetpoint = l;
        velRightSetpoint = r;
      }
    }

    // Informar o último incremento de posição
    // else if (cmd == 'O')
    // {
    //   Serial.print("O ");
    //   Serial.print(Pose[0], 6);
    //   Serial.print(", ");
    //   Serial.print(Pose[1], 6);
    //   Serial.print(", ");
    //   Serial.println(Pose[2], 6);

    //   noInterrupts();
    //   Pose[0] = 0.0;
    //   Pose[1] = 0.0;
    //   Pose[2] = 0.0;
    //   interrupts();
    // }
  }
}

// int convertePWM(float sinal)
// {
//   int pwm = sinal * 255 / maxVel;
//   int sig = abs(pwm)/ pwm;
//   pwm = constrain(abs(pwm), 0, 255) * sig;
//   return pwm;
// }

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
    while(!Serial){};
    
    PCISetup(ENC_LB);
    PCISetup(ENC_RB);

    pinMode(ENC_LA, INPUT_PULLUP);
    pinMode(ENC_LB, INPUT_PULLUP);
    pinMode(ENC_RA, INPUT_PULLUP);
    pinMode(ENC_RB, INPUT_PULLUP);

    pinMode(MRA, OUTPUT);
    pinMode(MRB, OUTPUT);
    pinMode(MLA, OUTPUT);
    pinMode(MLB, OUTPUT);

    // Serial.println("READY");
    // tLast = tCurr = millis();
}

void loop()
{

  unsigned long t = millis();
  if((t - tLastOdom) > odomInterval)
  {
    int pL = pulsosEncL, pR = pulsosEncR;
    pulsosEncL = 0;
    pulsosEncR = 0;

    float vL = calcSpeed(pL);
    float vR = calcSpeed(pR);

    float aL = speedControl(vL, velLeftSetpoint);
    float aR = speedControl(vR, velRightSetpoint);

    signalPWMLeft += aL;
    signalPWMRight += aR;

    acionarMotores(signalPWMLeft, signalPWMRight);

    tLastOdom = t;
    Serial.println();
  }


  ouvirSerial();
}

float calcSpeed(int pulses)
{
  static float odomIntervalms = odomInterval / 1000.0;
  float vel = pulse * deltaPhi / odomIntervalms * roda_raio;
  Serial.print(vel);
  Serial.print(", ");
  return vel;
}

float speedControl(float vel, float setpoint)
{
  static float lastError = 0;
  float error = setpoint - vel;
  float signal = kp * error + kd * ( lastError - error);
  lastError = error;
  return signal;
}