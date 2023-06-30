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

volatile long int pulsosEncL = 0, pulsosEncR = 0;

void PCISetup(byte pin);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
void ouvirSerial();
void acionarMotores(int pwmE, int pwmD);
void acionar(int pwm, int MA, int MB);

void ouvirSerial()
{
  if (Serial.available() > 0)
  {
    char cmd = Serial.read();

    if (cmd == 'V')
    {
      int r = Serial.parseFloat();
      int l = Serial.parseFloat();
        acionarMotores(l, r);
    }

    else if (cmd == 'O')
    {
      Serial.print("O ");
      Serial.print(pulsosEncL);
      
      Serial.print(", ");
      Serial.println(pulsosEncR);

      noInterrupts();
      pulsosEncL = 0;
      pulsosEncR = 0;
      interrupts();
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

    while(Serial.available()) Serial.read();
    Serial.println("READY");
    tLast = tCurr = millis();
}

void loop()
{

  ouvirSerial();

}

