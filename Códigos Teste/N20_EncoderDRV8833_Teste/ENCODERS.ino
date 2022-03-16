void handleLEncoder()
{
  chALState = digitalRead(encoderLChA);
  chBLState = digitalRead(encoderLChB);

  if (chALState != chBLState) {
    if (chALState != chALLastS) {
      posL++;
      chALLastS = chALState;
    }
    else if (chBLState != chBLLastS) {
      posL--;
      chBLLastS = chBLState;
    }
  }

  else {
    if (chALState != chALLastS) {
      posL--;
      chALLastS = chALState;
    }
    else if (chBLState != chBLLastS) {
      posL++;
      chBLLastS = chBLState;
    }
  }

}

void handleREncoder()
{
  chARState = digitalRead(encoderRChA);
  chBRState = digitalRead(encoderRChB);

  if (chARState != chBRState) {
    if (chARState != chARLastS) {
      posR--;
      chARLastS = chARState;
    }
    else if (chBRState != chBRLastS) {
      posR++;
      chBRLastS = chBRState;
    }
  }

  else {
    if (chARState != chARLastS) {
      posR++;
      chARLastS = chARState;
    }
    else if (chBRState != chBRLastS) {
      posR--;
      chBRLastS = chBRState;
    }
  }
}

void computarPosicao() {

  float passosMDir = float(posR) - float(lastPosR); // Dá a posição do motor observado em passos
  float passosMEsq = float(posL) - float(lastPosL);
  float dThetaDir = (passosMDir * distPasso) / raioRodas;  //Deslocamento angular da roda direita em radianos
  float dThetaEsq = (passosMEsq * distPasso) / raioRodas;  //Deslocamento angular da roda esquerda em radianos

  float dL = ((dThetaDir * raioRodas) + (dThetaEsq * raioRodas)) / 2; // ??? Média do deslocamento das duas rodas em mm
  float dTheta = ((dThetaDir * raioRodas) - (dThetaEsq * raioRodas)) / entreRodas; // ??? Posição angular em radianos

  if (dTheta != 0 ) {
    X = X + (dL / dTheta) * ( sin(Theta + dTheta) - sin(Theta) );
    Y = Y - (dL / dTheta) * ( cos(Theta + dTheta) - cos(Theta) );
    Theta = Theta  + dTheta;
  }
  else {
    X = X  + dL * cos(Theta);
    Y = Y  + dL * sin(Theta);
    Theta = Theta;
  }

  lastPosR = posR;
  lastPosL = posL;
}

void checkPos() {
  static unsigned long lastCheck = millis;
  unsigned long currentTime = millis();
  unsigned long time2CheckEncoders = currentTime - lastCheck;
  if (time2CheckEncoders > 1) {
    computarPosicao();

    lastCheck = currentTime;
  }
}
