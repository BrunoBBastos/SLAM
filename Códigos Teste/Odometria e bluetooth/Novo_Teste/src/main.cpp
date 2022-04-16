#include "Arduino.h"
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
float roda_raio = 31.75/1000;
int rpm = 100;
float l = 116.0/1000;
float dPhi = 2*PI / 2091.0;
float dt = 20.0/1000;
volatile long pulsosR = 0, pulsosL = 0;
volatile float x = 0.0, y = 0.0, theta = 0.0;
float k1 = 0.05, k2 = 0.20;

////////////////////////////////////////////////////// Testes

float referencia[2] = {1, 0};
char modo = 'm';

////////////////////////////////////////////////////// Protótipos

void PCISetup(byte pin);
void controleManual(int esquerdo, int direito);
ISR (PCINT0_vect); // Interrupt Service Routine dos pinos 8 a 13
ISR (PCINT2_vect); // Interrupt Service Routine dos pinos 0 a 7
void odometria();
float angleWrap(float ang);
void bluetoothControl();
void printOdom();
void mover(float referencia[2]);
void controleRef(float Xref, float Yref, float ctrl[2]);
int convertePWM(float sinal);
void modo_de_Operacao(char modo);

////////////////////////////////////////////////////// ROBÔ

class RoboUniciclo
{
  public:

  float pose[3] = {0.0f, 0.0f, 0.0f};

  void begin()
  {
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
    Serial.print("Acionando motor\nEnvie qqr coisa para parar\n");
    while(Serial.available() != 0) Serial.read();
  }

  float medirDistancia(float p[2])
  {
    return sqrt(pow(pose[0] - p[0], 2) + pow(pose[1] - p[1], 2));
  }

  bool seguirReferencia(float referencia[2])
  {
    float distancia = medirDistancia(referencia);
    if (distancia < 0.05) // Tolerância de 5 cm
    {
      Serial.print(distancia);
      Serial.println("m até o destino.");
      // printOdom();
      return true;
    }

    int ctrl[2];
    this -> gerarSinalControle(referencia, ctrl);
    this -> mover(ctrl);
    return false;
  }

  void gerarSinalControle(float referencia[2], int ctrl[2])
  {
    float dY = (referencia[1] - pose[1]);
    float dX = (referencia[0] - pose[0]);
    float THETAref = atan2(dY, dX);
    THETAref = angleWrap(THETAref);

    float erroAngular = angleWrap(THETAref - pose[2]);
    float erroLinear = sqrt(pow(dX, 2) + pow(dY, 2)) * cos(erroAngular);

    float v = k1 * erroLinear;
    float w = k2 * erroAngular;

    ctrl [0] = this -> convertePWM(v - w);
    ctrl [1] = this -> convertePWM(v + w);
  }

  int convertePWM(float sinal)
  {
    float maxVel = (rpm/60) * 2 * PI * roda_raio;
    int pwm = sinal * 255 / maxVel;
    int sig = abs(pwm)/ pwm;
    pwm = constrain(abs(pwm), 50, 255) * sig;
    return pwm;
  }

  void mover(int ctrl[2])
  {
    if (ctrl[0] < 0)
    {
      digitalWrite(MLA, 0);
      analogWrite(MLB, abs(ctrl[0]));
    }
    else
    {
      digitalWrite(MLB, 0);
      analogWrite(MLA, abs(ctrl[0]));
    }

    if (ctrl[1] < 0)
    {
      digitalWrite(MRB, 0);
      analogWrite(MRA, abs(ctrl[1]));
    }
    else
    {
      digitalWrite(MRA, 0);
      analogWrite(MRB, abs(ctrl[1]));
    }
  }

  void controleManual()
  {
    char buff = Serial.read();

    switch (buff) {

      case 'F':
        int ctrl[2] = {255, 255};
        this -> mover(ctrl);
        break;

      case 'B':
        int ctrl[2] = {-255, -255};
        this -> mover(ctrl);
        break;

      case 'L':
        int ctrl[2] = {-255, 255};
        this -> mover(ctrl);
        break;

      case 'R':
        int ctrl[2] = {255, -255};
        this -> mover(ctrl);
        break;

      case 'G':
        int ctrl[2] = {0, 255};
        this -> mover(ctrl);
        break;

      case 'I':
        int ctrl[2] = {255, 0};
        this -> mover(ctrl);
        break;

      case 'H':
        int ctrl[2] = {0, -255};
        this -> mover(ctrl);
        break;

      case 'J':
        int ctrl[2] = {-255, 0};
        this -> mover(ctrl);
        break;

      case 'S':
        int ctrl[2] = {0, 0};
        this -> mover(ctrl);
        break;

      default:
        break;
    }
  }

};

RoboUniciclo rob;
////////////////////////////////////////////////////// SETUP

void setup() {
  
  rob.begin();

  MsTimer2::set(dt*1000, odometria);
  MsTimer2::start();
}

////////////////////////////////////////////////////// LOOP

void loop()
{
  // modo_de_Operacao(modo);
  float ref[2] = {1, 1};
  rob.seguirReferencia(ref);

}

////////////////////////////////////////////////////// FUNÇÕES


void odometria()
{
  // Velocidade linear da roda: v = w*r
  float Vr = pulsosR * (dPhi / dt) * roda_raio;
  float Vl = pulsosL * (dPhi / dt) * roda_raio;

  pulsosL = 0;
  pulsosR = 0;
  
  if (Vr == Vl) // Se o robô estiver andando em linha reta:
  {
    x += Vr * cos(theta) * dt;
    y += Vr * sin(theta) * dt;
    return;
  }
  
  // Velocidade angular do robô: (Vr - Vl)/l
  float w = (Vr - Vl) / l;
  // Distância para o centro de curvatura:
  float R = (l / 2) * (Vr + Vl)/(Vr - Vl);
  // Posição do Centro de Curvatura Instantâneo:
  float CCIx = x - R * sin(theta);
  float CCIy = y + R * cos(theta);

  x = cos(dt*w) * (x - CCIx) - sin(dt*w) * (y - CCIy) + CCIx;
  y = sin(dt*w) * (x - CCIx) + cos(dt*w) * (y - CCIy) + CCIy;
  theta += dt*w;
  theta = angleWrap(theta);
}




void controleManual(int esquerdo, int direito)
{
 if (esquerdo == 1) {
    digitalWrite(MLA, 1);
    digitalWrite(MLB, 0);
  }
  else if (esquerdo == 0) {
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 0);
  }
  else if (esquerdo == -1) {
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 1);
  }
  if (direito == 1) {
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 1);
  }
  else if (direito == 0) {
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 0);
  }
  else if (direito == -1) {
    digitalWrite(MRA, 1);
    digitalWrite(MRB, 0);
  }
}

void printOdom()
{
  Serial.println(x);
  Serial.println(y);
  Serial.println(theta);
  Serial.println();
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
    pulsosL--;
  }
  else
  {
    pulsosL++;
  }
}

ISR (PCINT2_vect) // Interrupt Service Routine dos pinos 0 a 7
{
  bool A = digitalRead(ENC_RA);
  bool B = digitalRead(ENC_RB);

  if(A == B)
  {
    pulsosR++;
  }
  else
  {
    pulsosR--;
  }
}

void modo_de_Operacao(char modo)
{

  if(Serial.available() > 0)
  {
    if (Serial.peek() == 'x')
    {
      while(Serial.available() > 0)
      {
        Serial.read();
      }
      Serial.println("Escolha o modo de operação:");
      Serial.println("'m' - Controle manual");
      Serial.println("'r' - Controle por referência");

      while (Serial.available() == 0);
      modo = Serial.read();
      while(Serial.available() != 0) Serial.read();
      Serial.print("Modo selecionado: ");
      Serial.print(modo);

    }
  }

  switch (modo)
  {
    case 'r':
      mover(referencia);
      break;

    case 'm':
      bluetoothControl();
      break;

    case 's':
      //PARAR
      digitalWrite(MLA, 0);
      digitalWrite(MLB, 0);
      digitalWrite(MRA, 0);
      digitalWrite(MRB, 0);
      break;

    default:
    modo = 's';
      break;

  }
}