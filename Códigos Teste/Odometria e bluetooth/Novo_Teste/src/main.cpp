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

volatile long int pulsosEncL = 0, pulsosEncR = 0;

void PCISetup(byte pin);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
float angleWrap(float ang);

////////////////////////////////////////////////////// CLASSES


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

class RoboUniciclo
{
  public:
  float roda_raio = 31.75/1000;
  int rpm = 100;
  float l = 120.0/1000;
  float dPhi = 2*PI / 2091.0;
  float dt = 20.0/1000;
  float pose[3] = {0.0f, 0.0f, 0.0f};
  char modoOp = 'm';

  Motor MEsq, MDir;
  Encoder EEsq, EDir;

  static bool flagEnc;

  // Instanciar classes, inicializar pinos, etc
  void preparar()
  {
    Serial.begin(115200);

    MEsq.begin(MLA, MLB); 
    MDir.begin(MRB, MRA);

    EEsq.begin(ENC_LA, ENC_LB, dPhi, pulsosEncL);
    EDir.begin(ENC_RA, ENC_RB, dPhi, pulsosEncR);
    flagEnc = false;

    // Colocar dentro de uma função "iniciar teste" etc
    MsTimer2::set(dt*1000, RoboUniciclo::flagEncoders); 
    MsTimer2::start();
  }

  static void flagEncoders()
  {
    flagEnc = true;
  }

  char ouvirSerial()
  {
    if (Serial.available() > 0)
    {
      char cmd = Serial.read();
      while (Serial.available() > 0) Serial.read();

      switch(cmd)
      {
        case 'i':
          return 0;
          break;
        
        case 'x':
          acionarMotores(0, 0);
          Serial.println("Selecione o modo de operação:");
          Serial.println("t - Testar Motores");
          Serial.println("o - Testar Odometria");
          Serial.println("m - Controle Manual");
          Serial.flush();
          while(Serial.available() > 0) Serial.read();
          while(Serial.available() == 0);
          modoOp = Serial.read();
          Serial.println("Mensagem de confirmação"); // wip
          return 1;
          break;
        
        default:
          return cmd;
          break;
      }
    }
  }

  // Agir de acordo com o modo de execução selecionado
  void executar()
  {
    char cmd = ouvirSerial();

    switch(modoOp)
    {
      case 'm':
        controleManual(cmd);
        break;

      case 't':
        testarMotores();
        break;
      
      case 'o':
        testarOdometria();
        break;

      default:
        break;
    }
  }

  void controleManual(char cmd)
  {
    switch (cmd) {

      case 'F':
        acionarMotores(255, 255);
        break;

      case 'B':
        acionarMotores(-255, -255);
        break;

      case 'L':
        acionarMotores(-255, 255);
        break;

      case 'R':
        acionarMotores(255, -255);
        break;

      case 'G':
        acionarMotores(0, 255);
        break;

      case 'I':
        acionarMotores(255, 0);
        break;

      case 'H':
        acionarMotores(0, -255);
        break;

      case 'J':
        acionarMotores(-255, 0);
        break;

      case 'S':
        acionarMotores(0, 0);
        break;

      default:
        break;
    }
    
  }

  void acionarMotores(int pwmE, int pwmD)
  {
    MEsq.acionar(pwmE);
    MDir.acionar(pwmD);
  }

  void controleManual()
  {

  }

  void testarOdometria()
  {
    acionarMotores(150, 150);
    if(flagEnc)
    {
      float vE = EEsq.velocidadeAngRoda() * roda_raio;
      float vD = EDir.velocidadeAngRoda() * roda_raio;
      flagEnc = false;
      
      if (vE == vD)
      {
        pose[0] += vD * cos(pose[2]) * dt; 
        pose[1] += vD * sin(pose[2]) * dt;
      }

      else
      {
        float w = (vD - vE) / l;
        float R = (l/2) * (vD + vE) / (vD - vE);

        float CCIx = pose[0] - R * sin(pose[2]);
        float CCIy = pose[1] + R * cos(pose[2]);

        pose[0] = cos(dt * w) * (pose[0] - CCIx) - sin(dt*w) * (pose[1] - CCIy) + CCIx;
        pose[1] = sin(dt * w) * (pose[0] - CCIx) + cos(dt*w) * (pose[1] - CCIy) + CCIy;
        pose[2] = pose[2] + dt*w;
        pose[2] = angleWrap(pose[2]);

        Serial.print(pose[0]);
        Serial.print(' ');
        Serial.print(pose[1]);
        Serial.print(' ');
        Serial.println(pose[2]);
      }
    }
  }

  void testarMotores()
  {
    acionarMotores(150, 150);
    if(flagEnc)
    {
      Serial.print(EEsq.velocidadeAngRoda() * roda_raio);
      Serial.print(' ');
      Serial.println(EDir.velocidadeAngRoda() * roda_raio);
      flagEnc = false;
    }
  }
};

bool RoboUniciclo::flagEnc;


////////////////////////////////////////////////////// GLOBAIS

RoboUniciclo robo;


////////////////////////////////////////////////////// SETUP

void setup()
{
  robo.preparar();
  PCISetup(ENC_LB);
  PCISetup(ENC_RB);
}

////////////////////////////////////////////////////// LOOP


void loop()
{
  robo.executar();
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