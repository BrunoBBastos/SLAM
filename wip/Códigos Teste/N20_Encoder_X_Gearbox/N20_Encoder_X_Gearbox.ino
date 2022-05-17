#define M1    7   // Pinos de controle do motor
#define M2    6
#define ENC_A 12  // Pinos de leitura do encoder
#define ENC_B 13


//######################################################## ENCODER

volatile bool encAEstado, encAAnterior;
volatile long pulsos = 0;

//######################################################## SETUP

void setup() {
  Serial.begin(115200);
  
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);

  encAAnterior = digitalRead(ENC_A);
  
  PCISetup(ENC_A);

  Serial.println("Envie qqr coisa para acionar");
  
  while(Serial.available() == 0);
  Serial.print("Acionando motor\nEnvie qqr coisa para parar \n");
  while(Serial.available() != 0) Serial.read();
  pulsos = 0;
  digitalWrite(M1, 0);
  analogWrite(M2, 100);
}

void loop() {
  if(Serial.available() > 0)
  {
    Serial.println("Motores desligados");
    while(Serial.available() > 0) Serial.read();
    digitalWrite(M1, 0);
    digitalWrite(M2, 0);
    Serial.println("Alinhe a roda e envie o número de voltas contadas");
    while(Serial.available() == 0);
    int voltas = Serial.parseInt();
    Serial.print(pulsos/voltas);
    Serial.println(" pulsos por volta");
//    while(1); 
  }
}

void PCISetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
  PCIFR |= bit(digitalPinToPCICRbit(pin));
  PCICR |= bit(digitalPinToPCICRbit(pin));
}

void ISR(PCINT0_vect) // Interrupt Service Routine dos pinos 8 a 13
{
  encAEstado = digitalRead(ENC_A);
  bool B = digitalRead(ENC_B);

  if(encAEstado != encAAnterior)
  {
    if(encAEstado == B)
    {
      pulsos++;
    }
    else
    {
      pulsos--;
    }
    encAAnterior = encAEstado;
  }
}
