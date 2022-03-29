#define ENC_LA               13
#define ENC_LB               12
#define MRB                  10
#define MRA                  11
#define SERVO                9
#define ENC_RA               8
#define ENC_RB               7
#define MLA                  5
#define MLB                  6


//######################################################## ENCODER

const float ppr = 4182.0;
volatile bool encAEstado, encAAnterior;
volatile long pulsos = 0;

float roda_r = 43.0/1000;

//######################################################## SETUP

unsigned long tlast;

void setup() {
  Serial.begin(115200);
  
  pinMode(ENC_LB, INPUT_PULLUP);
  pinMode(ENC_LA, INPUT_PULLUP);
  pinMode(MLA, OUTPUT);
  pinMode(MLB, OUTPUT);
  pinMode(MRA, OUTPUT);
  pinMode(MRB, OUTPUT);

  encAAnterior = digitalRead(ENC_LB);
  
  PCISetup(ENC_LB);

  Serial.println("Envie qqr coisa para acionar");
  
  while(Serial.available() == 0);
  Serial.print("Acionando motor\nEnvie qqr coisa para parar \n");
  while(Serial.available() != 0) Serial.read();
  pulsos = 0;
  digitalWrite(MLB, 0);
  analogWrite(MLA, 255);
  digitalWrite(MRA, 0);
  analogWrite(MRB, 255);
  tlast = millis();
}

void loop() {
  if(Serial.available() > 0)
  {
    Serial.println("Motores desligados");
    while(Serial.available() > 0) Serial.read();
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 0);
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 0);
    float X = float(pulsos / ppr) * (2* PI * roda_r);
    
    Serial.println(X);
    Serial.println(pulsos);
    Serial.println(ppr);
    Serial.println(pulsos/ppr);
    Serial.println(2*PI*roda_r);
    
//    Serial.println("Alinhe a roda e envie o número de voltas contadas");
//    while(Serial.available() == 0);
//    int voltas = Serial.parseInt();
//    Serial.print(pulsos/voltas);
//    Serial.println(" pulsos por volta");
//    while(1); 
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
  encAEstado = digitalRead(ENC_LB);
  bool B = digitalRead(ENC_LA);

  if(encAEstado != encAAnterior)
  {
    if(encAEstado == B)
    {
      pulsos--;
    }
    else
    {
      pulsos++;
    }
    encAAnterior = encAEstado;
  }
}
