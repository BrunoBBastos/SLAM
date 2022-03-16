volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

void acc_gyrSetup() {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(accGyrInt, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1688); // 1788 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(accGyrInt));
    Serial.println(F(")..."));
    enableInterrupt(accGyrInt, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void receberAccGyr() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

    tempoIntAccGyr[1] = tempoIntAccGyr[0];
    tempoIntAccGyr[0] = micros();
    float deltaTempo = float(tempoIntAccGyr[0] - tempoIntAccGyr[1]) / 1000000L;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    //  mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    if (!chaveIntegrar) return;

    reciclarArray(aceleracao);
    reciclarArray(velocidade);
    reciclarArray(posicao);

    aceleracao[0][0] = converterAcc(aaWorld.x, accFatorEscala);
    aceleracao[0][1] = converterAcc(aaWorld.y, accFatorEscala);
    aceleracao[0][2] = converterAcc(aaWorld.z, accFatorEscala);

    for (int i = 0; i < 3; i++) {
      interpolar(velocidade, aceleracao, deltaTempo, i);
      interpolar(posicao, velocidade, deltaTempo, i);
    }
    //    printArray(posicao);

  }
}

float converterAcc(int acc, int scaleF) { // equivalente à função map() para floats
  return ((float(acc) + 32768.0f) * (2.0f * gravidade_mss * scaleF) / (32767 + 32768) - gravidade_mss * scaleF); // int max e min: 32767, -32768
}

void interpolar(float fx[2][3], float dx[2][3], float t, int n) {
  //  t/=1000.0f;
  fx[0][n] = fx[1][n] + dx[0][n] * t - ((dx[0][n] - dx[1][n]) / 2) * t;
}

void reciclarArray(float arr[2][3]) {
  for (int i = 0; i < 3; i++) {
    arr[1][i] = arr[0][i];
  }
}

void printArray(float arr[2][3]) {
  for (int i = 0; i < 3; i++) {
    Serial.print(arr[0][i]);
    Serial.print("\t");
  }
  Serial.println();
}
