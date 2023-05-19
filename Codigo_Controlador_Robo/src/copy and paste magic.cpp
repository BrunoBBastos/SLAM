// ////////////////////////////////////////////////////// BIBLIOTECAS

// #include "Arduino.h"
// #include "MsTimer2.h"

// ////////////////////////////////////////////////////// DEFINES

// #define ENC_LA               13
// #define ENC_LB               12
// #define MRB                  10
// #define MRA                  11
// #define SERVO                9
// #define ENC_RA               8
// #define ENC_RB               7
// #define MLA                  5
// #define MLB                  6

// volatile long int pulsosEncL = 0, pulsosEncR = 0;

// void PCISetup(byte pin);
// ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
// ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
// float angleWrap(float ang);
// double distancia2D(float p1[2], float p2[2]);


// ////////////////////////////////////////////////////// CLASSES

// class Controle
// {
//   public:
//   float kpLinear;
//   float kpAngular;
//   float ref[3];

//   void begin(float kpL, float kpA)
//   {
//     kpLinear = kpL;
//     kpAngular = kpA;
//   }

//   void mudarConstantes(float kpL, float kpA)
//   {
//     kpLinear = kpL;
//     kpAngular = kpA;
//   }

//   void receberReferencia(float x, float y, float t)
//   {
//     ref[0] = x;
//     ref[1] = y;
//     ref[2] = t;
//   }

//   void gerarSinalControlePosicao(float pose[3], float referencia[3], float ctrl[2])
//   {
//     float dY = (referencia[1] - pose[1]);
//     float dX = (referencia[0] - pose[0]);
//     float THETAref = atan2(dY, dX);
//     THETAref = angleWrap(THETAref);

//     float erroAngular = angleWrap(THETAref - pose[2]);
//     float erroLinear = sqrt(pow(dX, 2) + pow(dY, 2)) * cos(erroAngular);

//     float v = kpLinear * erroLinear;
//     float w = kpAngular * erroAngular;

//     velocidadeCombinarComponentes(v, w, ctrl);
    
//   }

//   void velocidadeCombinarComponentes(float linear, float angular, float vw[2])
//   {
//     vw[0] = linear - angular;
//     vw[1] = linear + angular;
//   }
// };

// class Motor
// {

//   int A, B;

//   public:
//   void begin(int pinA, int pinB)
//   {
//     A = pinA;
//     B = pinB;
//     pinMode(A, OUTPUT);
//     pinMode(B, OUTPUT);
//   }

//   void acionar(int pwm)
//   {
//     if (pwm > 0)
//     {
//       analogWrite(A, pwm);
//       digitalWrite(B, 0);
//     }
//     else{
//       analogWrite(A, 0);
//       digitalWrite(B, pwm);  
//     }
//   }
// };

// class Encoder
// {

//   int A, B;
//   volatile long int *pulsos;
//   float dPhi;
//   float dt = 20.0f/1000;

//   public:
//   void begin(int pinA, int pinB, float deltaPhi, volatile long int &varPulsos)
//   {
//     A = pinA;
//     B = pinB;
//     dPhi = deltaPhi;
//     pulsos = &varPulsos;
//     pinMode(A, INPUT_PULLUP);
//     pinMode(B, INPUT_PULLUP);
//   }

//   // Observa a contagem de pulsos sem resetar o valor
//   long int contar()
//   {
//     return *pulsos;
//   }

//   // Observa a contagem de pulsos e reseta o valor
//   long int coletarPulsos()
//   {
//     long int total = *pulsos;
//     *pulsos = 0;
//     return total;
//   }

//   float velocidadeAngRoda()
//   {
//     return coletarPulsos() * dPhi/dt;
//   }

// };

// class RoboUniciclo
// {
//   public:
//   float roda_raio = 31.75/1000;
//   int rpm = 100;
//   float l = 120.0/1000;
//   float dPhi = 2*PI / 2091.0;
//   float dt = 20.0/1000;
//   float pose[3] = {0.0f, 0.0f, 0.0f};
//   char modoOp = 'i';

//   Motor MEsq, MDir;
//   Encoder EEsq, EDir;
//   Controle CRef;

//   static bool flagEnc;

//   // Instanciar classes, inicializar pinos, etc
//   void preparar()
//   {
//     Serial.begin(115200);

//     MEsq.begin(MLA, MLB); 
//     MDir.begin(MRB, MRA);

//     EEsq.begin(ENC_LA, ENC_LB, dPhi, pulsosEncL);
//     EDir.begin(ENC_RA, ENC_RB, dPhi, pulsosEncR);
//     flagEnc = false;

//     CRef.begin(0.5, 0.2);

//     // Colocar dentro de uma função "iniciar teste" etc
//     MsTimer2::set(dt*1000, RoboUniciclo::flagEncoders); 
//     MsTimer2::start();
//   }

//   // Sinalizar que os encoders coletaram pulsos no intervalo dt
//   static void flagEncoders()
//   {
//     flagEnc = true;
//   }

//   void ouvirSerial()
//   {
//     int cmd;
//     if (Serial.available() > 0)
//     {
//       cmd = Serial.read();
//       Serial.println(cmd);

//       switch(cmd)
//       {
//         // Receber componentes lineares e angulares de velocidade
//         case 'V':
//          {
//            Serial.println("Recebendo velocidades");
//             modoOp = 'V';
//             float v = Serial.parseFloat(SKIP_WHITESPACE);
//             float w = Serial.parseFloat(SKIP_WHITESPACE);
//             float ctrl[2];
            

//             CRef.velocidadeCombinarComponentes(v, w, ctrl);

//             ctrl[0] = convertePWM(ctrl[0]);
//             ctrl[1] = convertePWM(ctrl[1]);

//             acionarMotores(ctrl[0], ctrl[1]);
//           }
//           break;
        
//         // Atualizar posição
//         case 'P':
//         {
//             Serial.println("Recebendo nova posição");
//             modoOp = 'P';
//             pose[0] = Serial.parseFloat(SKIP_WHITESPACE);
//             pose[1] = Serial.parseFloat(SKIP_WHITESPACE);
//             pose[2] = Serial.parseFloat(SKIP_WHITESPACE);
//             Serial.println("Pose Atualizada:");
//             for(int i = 0; i < 3; i++) Serial.println(pose[i]);
//         }
//           break;

//         // Receber novo objetivo de posição
//         case 'R':
//           {
//           Serial.println("Recebendo referência");
//           modoOp = 'R';
//           float x = Serial.parseFloat(SKIP_WHITESPACE);
//           float y = Serial.parseFloat(SKIP_WHITESPACE);
//           float t = Serial.parseFloat(SKIP_WHITESPACE);
//           CRef.receberReferencia(x, y, t);
//           float p1[2] = {pose[0], pose[1]};
//           float p2[2] = {CRef.ref[0], CRef.ref[1]};
//           Serial.print(distancia2D(p1, p2)); // Debugging
//           Serial.println(" metros"); // Debugging
//           }
//           break;
        
//         case 'M':
//           {
//             acionarMotores(0, 0);
//           }
//           break;

//         case 'O':
//           {
//             imprimirOdometria();
//           }
//           break;

//       }
//     }
//   }

//   void imprimirOdometria()
//   {
//     Serial.print("O ");
//     Serial.print(pose[0]);
//     Serial.print(' ');
//     Serial.print(pose[1]);
//     Serial.print(' ');
//     Serial.println(pose[2]);
//   }

//   // Agir de acordo com o modo de execução selecionado
//   void executar()
//   {

//     ouvirSerial();
//     if(flagEnc)
//     {
//       odometria();
//     }

//     switch(modoOp)
//     {
//       case 't':
//         testarMotores();
//         break;
      
//       case 'M':
//         acionarMotores(0, 0);
//         break;

//       case 'R':
//         seguirRef();
//         break;

//       case 'V':
//         break;
        
//       case 'P':
//         break;

//       default:
//         break;
//     }
//   }

//   int convertePWM(float sinal)
//   {
//     float maxVel = (rpm/60) * 2 * PI * roda_raio;
//     int pwm = sinal * 255 / maxVel;
//     int sig = abs(pwm)/ pwm;
//     pwm = constrain(abs(pwm), 0, 255) * sig;
//     return pwm;
//   }

//   bool seguirRef()
//   {
//     float pos[2] = {pose[0], pose[1]};
//     float ref[2] = {CRef.ref[0], CRef.ref[1]};
//     float dist = distancia2D(pos, ref); 
//     if(dist > 0.05)
//     {
//       float ctrl[2];
//       CRef.gerarSinalControlePosicao(pose, CRef.ref, ctrl);
//       ctrl[0] = convertePWM(ctrl[0]);
//       ctrl[1] = convertePWM(ctrl[1]);
//       acionarMotores(ctrl[0], ctrl[1]);
//       return 1;
//     }
//     else{
//       Serial.print("A ");
//       Serial.print(dist);
//       Serial.println(" m de distância do alvo");
//       modoOp = 'M';
//     }
//     return 0;
//   }

//   void acionarMotores(int pwmE, int pwmD)
//   {
//     MEsq.acionar(pwmE);
//     MDir.acionar(pwmD);
//   }

//   void odometria()
//   {
//     if(flagEnc)
//     {
//       float vE = EEsq.velocidadeAngRoda() * roda_raio;
//       float vD = EDir.velocidadeAngRoda() * roda_raio;
//       flagEnc = false;
      
//       if (vE == vD)
//       {
//         pose[0] += vD * cos(pose[2]) * dt; 
//         pose[1] += vD * sin(pose[2]) * dt;
//       }

//       else
//       {
//         float w = (vD - vE) / l;
//         float R = (l/2) * (vD + vE) / (vD - vE);

//         float CCIx = pose[0] - R * sin(pose[2]);
//         float CCIy = pose[1] + R * cos(pose[2]);


//         // Coletar apenas o incremento - wip
//         pose[0] = cos(dt * w) * (pose[0] - CCIx) - sin(dt*w) * (pose[1] - CCIy) + CCIx;
//         pose[1] = sin(dt * w) * (pose[0] - CCIx) + cos(dt*w) * (pose[1] - CCIy) + CCIy;
//         pose[2] = pose[2] + dt*w;
//         pose[2] = angleWrap(pose[2]);
//       }
//     }
//   }

//   void testarMotores()
//   {
//     acionarMotores(150, 150);
//     if(flagEnc)
//     {
//       Serial.print(EEsq.velocidadeAngRoda() * roda_raio);
//       Serial.print(' ');
//       Serial.println(EDir.velocidadeAngRoda() * roda_raio);
//       flagEnc = false;
//     }
//   }
// };

// bool RoboUniciclo::flagEnc;

// ////////////////////////////////////////////////////// GLOBAIS

// RoboUniciclo robo;

// ////////////////////////////////////////////////////// SETUP

// void setup()
// {
//   robo.preparar();
//   PCISetup(ENC_LB);
//   PCISetup(ENC_RB);
// }

// ////////////////////////////////////////////////////// LOOP


// void loop()
// {
//   robo.executar();
// }

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
//     pulsosEncL--;
//   }
//   else
//   {
//     pulsosEncL++;
//   }
// }

// ISR (PCINT2_vect) // Interrupt Service Routine dos pinos 0 a 7
// {
//   bool A = digitalRead(ENC_RA);
//   bool B = digitalRead(ENC_RB);

//   if(A == B)
//   {
//     pulsosEncR++;
//   }
//   else
//   {
//     pulsosEncR--;
//   }
// }

// float angleWrap(float ang)
// {
//   if (ang > PI)
//   {
//     return ang - 2 * PI;
//   }
//   else if (ang < -PI)
//   {
//     return ang + 2 * PI;
//   }
//   return ang;
// }

// double distancia2D(float p1[2], float p2[2])
// {
//   return sqrt(pow(p2[0] - p1[0], 2) + pow(p2[1] - p1[1], 2));
// }