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
double Pose[3] = {0.0, 0.0, 0.0}; 
volatile long int pulsosEncL = 0, pulsosEncR = 0;

void PCISetup(byte pin);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
float angleWrap(float ang);
double distancia2D(float p1[2], float p2[2]);


void setup()
{
    Serial.begin(115200);

    PCISetup(ENC_LB);
    PCISetup(ENC_RB);
}


void loop()
{

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

void ouvirSerial()
  {
    int cmd;
    if (Serial.available() > 0)
    {
      cmd = Serial.read();
      Serial.println(cmd);

      switch(cmd)
      {
        // Receber componentes lineares e angulares de velocidade
        case 'V':
         {
           Serial.println("Recebendo velocidades");
            modoOp = 'V';
            float v = Serial.parseFloat(SKIP_WHITESPACE);
            float w = Serial.parseFloat(SKIP_WHITESPACE);
            float ctrl[2];
            

            CRef.velocidadeCombinarComponentes(v, w, ctrl);

            ctrl[0] = convertePWM(ctrl[0]);
            ctrl[1] = convertePWM(ctrl[1]);

            acionarMotores(ctrl[0], ctrl[1]);
          }
          break;
        
        // // Atualizar posição
        // case 'P':
        // {
        //     Serial.println("Recebendo nova posição");
        //     modoOp = 'P';
        //     pose[0] = Serial.parseFloat(SKIP_WHITESPACE);
        //     pose[1] = Serial.parseFloat(SKIP_WHITESPACE);
        //     pose[2] = Serial.parseFloat(SKIP_WHITESPACE);
        //     Serial.println("Pose Atualizada:");
        //     for(int i = 0; i < 3; i++) Serial.println(pose[i]);
        // }
        //   break;

        // // Receber novo objetivo de posição
        // case 'R':
        //   {
        //   Serial.println("Recebendo referência");
        //   modoOp = 'R';
        //   float x = Serial.parseFloat(SKIP_WHITESPACE);
        //   float y = Serial.parseFloat(SKIP_WHITESPACE);
        //   float t = Serial.parseFloat(SKIP_WHITESPACE);
        //   CRef.receberReferencia(x, y, t);
        //   float p1[2] = {pose[0], pose[1]};
        //   float p2[2] = {CRef.ref[0], CRef.ref[1]};
        //   Serial.print(distancia2D(p1, p2)); // Debugging
        //   Serial.println(" metros"); // Debugging
        //   }
        //   break;
        
        // case 'M':
        //   {
        //     acionarMotores(0, 0);
        //   }
        //   break;

        // case 'O':
        //   {
        //     imprimirOdometria();
        //   }
        //   break;

      }
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