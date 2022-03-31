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
  else
  {
    return ang;
  }
}
