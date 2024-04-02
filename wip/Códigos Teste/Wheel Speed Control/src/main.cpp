#include "Arduino.h"
////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// PINAGEM

#define ENC_LA               13
#define ENC_LB               12
#define MRB                  10
#define MRA                  11
#define SERVO                9
#define ENC_RA               8
#define ENC_RB               7
#define MLA                  5
#define MLB                  6

////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////// PARÂMETROS
// ROBÔ
const double roda_raio = 31.75/1000;
const double deltaRotacao = 2*PI / 2091.0;
const double RPM = 100.0;
const double maxVel = (RPM/60.0) * 2 * PI * roda_raio;

// FUNCIONALIDADES
int dt = 25; // Período de amostragem
float dtms = dt/1000.0;
float Pose[3] = {0.0, 0.0, 0.0};

////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////// CONTROLE
float kp = 1, kd = 1;
unsigned long tLastOdom = 0;
float velLeftSetpoint = 0.2, velRightSetpoint = 0.0;

volatile long int pulsosEncL = 0, pulsosEncR = 0;

////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////// PROTÓTIPOS
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
void zerarEncoders();

////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////// FUNÇÕES
void zerarEncoders()
{
    pulsosEncL = 0;
    pulsosEncR = 0;
}

void amostrarVelocidades(int pulsosEsq, int pulsosDir, float& velEsq, float& velDir)
{
    velEsq = pulsosEsq * deltaRotacao/dtms * roda_raio;
    velDir = pulsosDir * deltaRotacao/dtms * roda_raio;
}

void calcularPoseRelativa(float velEsq, float velDir, float desloc[3])
{
    if (velEsq == velDir)
    {
        desloc[0] += velEsq * cos(desloc[2]) * dt; 
        desloc[1] += velEsq * sin(desloc[2]) * dt;
    }

    else
    {
        float w = (vD - vE) / l; // Velocidade angular
        float R = (l/2) * (vD + vE) / (vD - vE); // Raio de curvatura
        // Posição do centro de curvatura instantâneo
        float CCIx = desloc[0] - R * sin(desloc[2]);
        float CCIy = desloc[1] + R * cos(desloc[2]);

        desloc[0] = cos(dt*w) * (desloc[0]-CCIx) - sin(dt*w) * (desloc[1]-CCIy) + CCIx;
        desloc[1] = sin(dt*w) * (desloc[0]-CCIx) + cos(dt*w) * (desloc[1]-CCIy) + CCIy;
        desloc[2] = angleWrap(desloc[2] + dt*w);
    }
}

void odometria()
{
    float vE, vD;
    int pE = pulsosEncL;
    int pD = pulsosEncR;
    zerarEncoders();
    amostrarVelocidades(pE, pD, vE, vD);
    calcularPoseRelativa(vE, vD, Pose);
}

void limparSerialNL()
{
    while(Serial.available() > 0)
    {
        if(Serial.read() == '\n') break;
    }
}

void ouvirSerial()
{
    if(Serial.available() > 0)
    {
        char cmd = Serial.read();
        if (cmd == 'V')
        {
            try
            {
                float l = Serial.parseFloat();
                float r = Serial.parseFloat();
                char nl = Serial.read();
                if (!isnan(l) && !isnan(r) && nl == '\n')
                {
                    velLeftSetpoint = l;
                    velRightSetpoint = r;
                }
                else limparSerialNL();

            }
            catch(const exception& e)
            {
                limparSerialNL();
            }
        }
    }
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

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////// SETUP
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

    Serial.println("READY");
}

////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////// LOOP
void loop()
{
    
  unsigned long t = millis();

  if((t - tLastOdom) > dt)
  {
    
    float vE, vD;
    static float erroAntvE = 0, erroAntvD = 0;
    int pE = pulsosEncL;
    int pD = pulsosEncR;

    zerarEncoders();
    amostrarVelocidades(pE, pD, vE, vD);

    float aE = speedControl(vE, velLeftSetpoint);
    float aD = speedControl(vD, velRightSetpoint);

    pwmEsq += aE;
    pwmDir += aD;

    acionarMotores(pwmEsq, pwmDir);

    tLastOdom = t;
  }


  ouvirSerial();
}

float speedControl(float vel, float setpoint, float& erroAnt)
{
  float erro = setpoint - vel;
  float signal = kp * erro + kd * ( erroAnt - erro);
  erroAnt = erro;
  return signal;
}