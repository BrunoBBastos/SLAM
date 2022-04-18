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
  long int pulsos = 0;

  public:
  void begin(int pinA, int pinB)
  {
    A = pinA;
    B = pinB;
    pinMode(A, INPUT_PULLUP);
    pinMode(B, INPUT_PULLUP);
  }

};

class RoboUniciclo
{
  float roda_raio = 31.75/1000;
  int rpm = 100;
  float l = 120.0/1000;
  float dPhi = 2*PI / 2091.0;
  float dt = 20.0/1000;

  char _modo = 't';

  Motor Esq, Dir;

  public:

  // Instanciar classes, inicializar pinos, etc
  void preparar()
  {
    Serial.begin(115200);
    Esq.begin(MLA, MLB); 
    Dir.begin(MRB, MRA); 

  }
  // Agir de acordo com o modo de execução selecionado
  void executar()
  {
    switch(_modo)
    {
      case 'm':
        break;

      case 't':
        testarMotores();
        break;
      
      default:
        break;
    }
  }

  void acionarMotores(int pwmE, int pwmD)
  {
    Esq.acionar(pwmE);
    Dir.acionar(pwmD);
  }

  void testarMotores()
  {
    acionarMotores(255, 255);
  }
};



////////////////////////////////////////////////////// GLOBAIS

RoboUniciclo robo;

////////////////////////////////////////////////////// SETUP

void setup()
{
  robo.preparar();
}

////////////////////////////////////////////////////// LOOP


void loop()
{
  robo.executar();
}

// void PCISetup(byte pin)
// {
//   *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
//   PCIFR |= bit(digitalPinToPCICRbit(pin));
//   PCICR |= bit(digitalPinToPCICRbit(pin));
// }

// ISR (PCINT0_vect) // Interrupt Service Routine dos pinos 8 a 13
// {
//   bool A = digitalRead(ENC_LA);
//   bool B = digitalRead(ENC_LB);


//   if(A == B)
//   {
//     robo.incrementarPulsos(-1, 0);
//     // pulsosL--;
//   }
//   else
//   {
//     pulsosL++;
//   }
// }

// ISR (PCINT2_vect) // Interrupt Service Routine dos pinos 0 a 7
// {
//   bool A = digitalRead(ENC_RA);
//   bool B = digitalRead(ENC_RB);

//   if(A == B)
//   {
//     pulsosR++;
//   }
//   else
//   {
//     pulsosR--;
//   }
// }