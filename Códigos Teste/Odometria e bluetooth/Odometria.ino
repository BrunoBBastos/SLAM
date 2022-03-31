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

// Colocar dentro do menu 'modo de operação'
  Serial.println("Envie qqr coisa para acionar");
  while(Serial.available() == 0);
  Serial.print("Acionando motor\nEnvie qqr coisa para parar \n");
  while(Serial.available() != 0) Serial.read();

//  digitalWrite(MLA, 1);
//  digitalWrite(MLB, 0);
//  digitalWrite(MRA, 0);
//  digitalWrite(MRB, 1);

  MsTimer2::set(dt*1000, odometria);
  MsTimer2::start();
//  tPrev = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available() > 0)
  {
    
    bluetoothControl();
      
//    digitalWrite(MLA, 0);
//  
//    digitalWrite(MRB, 0);
//    printOdom();
  }
}

////////////////////////////////////////////////////// FUNÇÕES

void PCISetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}
