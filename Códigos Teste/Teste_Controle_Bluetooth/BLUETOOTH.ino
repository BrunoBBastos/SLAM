void bluetoothControl() {
  char buff = Serial.read();

  switch (buff) {

    case 'F':
      direcao(1, 1);
      break;

    case 'B':
      direcao(-1, -1);
      break;

    case 'L':
      direcao(-1, 1);
      break;

    case 'R':
      direcao(1, -1);
      break;

    case 'G':
      direcao(0, 1);
      break;

    case 'I':
      direcao(1, 0);
      break;

    case 'H':
      direcao(0, -1);
      break;

    case 'J':
      direcao(-1, 0);
      break;

    case 'S':
      direcao(0, 0);
      break;

    default:
      break;
   
  }
}
