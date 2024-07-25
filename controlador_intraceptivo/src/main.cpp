#include "Arduino.h"

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PINAGEM

#define ENC_LA 13
#define ENC_LB 12
#define MRB 10
#define MRA 11
#define SERVO 9
#define ENC_RA 8
#define ENC_RB 7
#define MLA 5
#define MLB 6

enum Estado
{
  CONTROLADOR,
  PWM,
  ESPERANDO
};

Estado modo_operacao = CONTROLADOR;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARÂMETROS

// ROBÔ
const float roda_raio = 63.7 / 2000.0f;
const float roda_base = 120 / 1000.0f;
const float deltaRotacao = 2 * PI / 2091.0f;
const float RPM = 100.0f;
const float maxVel = (RPM / 60.0f) * 2 * PI * roda_raio;

// FUNCIONALIDADES
unsigned int dtms = 25; // Período de amostragem
float dt = dtms / 1000.0f;
double Pose[3] = {0.0f, 0.0f, 0.0f};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROLE

unsigned long tLastOdom = 0;
// unsigned long tLastControl = 0;
float filtroDerivativo = 0.05f;
float kp_vel = 150.0f;
float ki_vel = 3000.0f;
float kd_vel = 5.0f;
float velLeftSetpoint = 0.0, velRightSetpoint = 0.0;

volatile long int pulsosEncL = 0, pulsosEncR = 0;
float pwmEsq = 0, pwmDir = 0;

typedef struct
{
  // ganhos
  float kp;
  float ki;
  float kd;

  // intervalo de amostragem
  float T;

  // filtro LP
  float tau;

  // limites de saída
  float limMax;
  float limMin;

  // memória
  float integrador;
  float erroAnt;
  float derivador;
  float medidaAnt;

  // saída
  float output;
} ControlePID;

void ControlePID_ini(ControlePID *pid);
float ControlePID_atualizar(ControlePID *pid, float setpoint, float medida);

void ControlePID_ini(ControlePID *pid)
{
  pid->integrador = 0.0f;
  pid->derivador = 0.0f;
  pid->erroAnt = 0.0f;
  pid->medidaAnt = 0.0f;
}

float ControlePID_atualizar(ControlePID *pid, float setpoint, float medida)
{
  float erro = setpoint - medida;
  // computando o termo proporcional
  float proporcional = pid->kp * erro;

  // computando o termo integrativo
  pid->integrador += 0.5f * pid->ki * pid->T * (erro + pid->erroAnt);
  // rotina anti-windup
  float limMinInt, limMaxInt;
  if (pid->limMax > proporcional)
    limMaxInt = pid->limMax - proporcional;
  else
    limMaxInt = 0.0f;
  if (pid->limMin < proporcional)
    limMinInt = pid->limMin - proporcional;
  else
    limMinInt = 0.0f;
  // saturador
  if (pid->integrador > limMaxInt)
    pid->integrador = limMaxInt;
  else if (pid->integrador < limMinInt)
    pid->integrador = limMinInt;
                                  
  // computando o termo derivativo
  pid->derivador = -(2.0f * pid->kd * (medida - pid->medidaAnt) + (2.0f * pid->tau - pid->T) * pid->derivador) / (2.0f * pid->tau + pid->T);

  // computando a saída
  pid->output = proporcional + pid->integrador + pid->derivador;
  if (pid->output > pid->limMax)
    pid->output = pid->limMax;
  else if (pid->output < pid->limMin)
    pid->output = pid->limMin;

  // memorizando o último update
  pid->erroAnt = erro;
  pid->medidaAnt = medida;
                                  // Serial.print("out: ");
                                  // Serial.print(pid->output, 4);
                                  // Serial.print("\t");

  return pid->output;
}

ControlePID pidEsquerdo = {kp_vel, ki_vel, kd_vel,
                           dt, filtroDerivativo,
                           1000, -1000};
ControlePID pidDireito = {kp_vel, ki_vel, kd_vel,
                          dt, filtroDerivativo,
                          1000, -1000};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PROTÓTIPOS

void PCISetup(byte pin);
ISR(PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR(PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
float angleWrap(float ang);
double distancia2D(float p1[2], float p2[2]);
void odometria(float &vE, float &vD);
void ouvirSerial();
void acionarMotores(int pwmE, int pwmD);
void acionar(int pwm, int MA, int MB);
void amostrarVelocidades(float &velEsq, float &velDir);
void zerarEncoders();
void zerarOdometria();
void printOdometria();
void printVelocidades(float vE, float vD);
float speedControl(float vel, float setpoint, float &erroAnt);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNÇÕES & UTILIDADES

void zerarEncoders()
{
  pulsosEncL = 0;
  pulsosEncR = 0;
}

void zerarOdometria()
{
  Pose[0] = 0;
  Pose[1] = 0;
  Pose[2] = 0;
}

void amostrarVelocidades(float &velEsq, float &velDir)
{
  int pE = pulsosEncL;
  int pD = pulsosEncR;
  zerarEncoders();
  velEsq = pE * deltaRotacao / dt * roda_raio;
  velDir = pD * deltaRotacao / dt * roda_raio;
}

void calcularPoseRelativa(float velEsq, float velDir, double desloc[3])
{
  if (velEsq == velDir)
  {
    desloc[0] += velEsq * cos(desloc[2]) * dt;
    desloc[1] += velEsq * sin(desloc[2]) * dt;
  }

  else
  {
    float w = (velDir - velEsq) / roda_base;                           // Velocidade angular
    float R = (roda_base / 2) * (velDir + velEsq) / (velDir - velEsq); // Raio de curvatura
    // Posição do centro de curvatura instantâneo
    float CCIx = desloc[0] - R * sin(desloc[2]);
    float CCIy = desloc[1] + R * cos(desloc[2]);

    desloc[0] = cos(dt * w) * (desloc[0] - CCIx) - sin(dt * w) * (desloc[1] - CCIy) + CCIx;
    desloc[1] = sin(dt * w) * (desloc[0] - CCIx) + cos(dt * w) * (desloc[1] - CCIy) + CCIy;
    desloc[2] = angleWrap(desloc[2] + dt * w);
  }
}

void odometria(float &vE, float &vD)
{
  amostrarVelocidades(vE, vD);
  calcularPoseRelativa(vE, vD, Pose);
}

void limparSerialNL()
{
  while (Serial.available() > 0)
  {
    if (Serial.read() == '\n')
      break;
  }
}

void ouvirSerial()
{

  if (Serial.available() > 0)
  {
    char cmd = Serial.read();

    if (cmd == 'V')
    {
      float l = Serial.parseFloat();
      float r = Serial.parseFloat();
      char nl = Serial.read();
      if (!isnan(l) && !isnan(r) && nl == '\n')
      {
        velLeftSetpoint = l;
        velRightSetpoint = r;
      }
      else
        limparSerialNL();
    }

    else if (cmd == 'P')
    {
      float l = Serial.parseInt();
      float r = Serial.parseInt();
      char nl = Serial.read();
      if (!isnan(l) && !isnan(r) && nl == '\n')
      {
        pwmEsq = l;
        pwmDir = r;
      }
      else
      {
        limparSerialNL();
      }
    }

    else if(cmd == 'M')
    {
      Estado modo = Serial.parseInt();
      char nl = Serial.read();
      if(nl == '\n')
      {
        switch(modo)
        {
          // É necessário fazer a limpeza das variáveis relevantes ao trocar de modo
          case PWM:
          {
            modo_operacao = modo;
            break;
          }

          case CONTROLADOR:
          {
            modo_operacao = modo;
            break;
          }

          case ESPERANDO:
          {
            modo_operacao = modo;
            break;
          }
        }
      }
    }
    
    else if (cmd == 'Z')
    {
      zerarOdometria();
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
  else
  {
    digitalWrite(MA, 0);
    analogWrite(MB, abs(pwm));
  }
}

void PCISetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

ISR(PCINT0_vect) // Interrupt Service Routine dos pinos 8 a 13
{
  bool A = digitalRead(ENC_LA);
  bool B = digitalRead(ENC_LB);

  if (A == B)
  {
    pulsosEncL--;
  }
  else
  {
    pulsosEncL++;
  }
}

ISR(PCINT2_vect) // Interrupt Service Routine dos pinos 0 a 7
{
  bool A = digitalRead(ENC_RA);
  bool B = digitalRead(ENC_RB);

  if (A == B)
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

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SETUP

void setup()
{
  Serial.begin(115200);
  while (!Serial);

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

  ControlePID_ini(&pidEsquerdo);
  ControlePID_ini(&pidDireito);

}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP

void loop()
{
  ouvirSerial();

  unsigned long t = millis();

  if(t - tLastOdom >= dtms)
  {
    float vE, vD;
    odometria(vE, vD);
    // printVelocidades(vE, vD);
    printOdometria();
    tLastOdom = t;

    switch(modo_operacao)
    {
      case CONTROLADOR:
      {
        float aE = ControlePID_atualizar(&pidEsquerdo, velLeftSetpoint, vE);
        float aD = ControlePID_atualizar(&pidDireito, velRightSetpoint, vD);

        // pwmEsq += aE; //orig
        // pwmEsq = constrain(pwmEsq, -255, 255); //orig
        pwmEsq = constrain(aE, -255, 255); //fake
        // pwmDir = aD;
        pwmDir = constrain(aD, -255, 255);
        acionarMotores((int)pwmEsq, (int)pwmDir);

        break;
      }

      case PWM:
      {
        acionarMotores((int)pwmEsq, (int)pwmDir);
        break;
      }

      case ESPERANDO:
      {
        Serial.println("READY");
        modo_operacao = PWM;
        break;
      }
    }
  }
}

void printVelocidades(float vE, float vD)
{
  Serial.print(vE, 6);
  Serial.print('\t');
  Serial.println(vD, 6);
}

void printOdometria()
{
  Serial.print("O ");
  Serial.print(Pose[0], 6);
  Serial.print(" ");
  Serial.print(Pose[1], 6);
  Serial.print(" ");
  Serial.println(Pose[2], 6);
}