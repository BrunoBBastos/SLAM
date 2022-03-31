
void bluetoothControl() {
  char buff = Serial.read();

  switch (buff) {

    case 'F':
      controleManual(1, 1);
      break;

    case 'B':
      controleManual(-1, -1);
      break;

    case 'L':
      controleManual(-1, 1);
      break;

    case 'R':
      controleManual(1, -1);
      break;

    case 'G':
      controleManual(0, 1);
      break;

    case 'I':
      controleManual(1, 0);
      break;

    case 'H':
      controleManual(0, -1);
      break;

    case 'J':
      controleManual(-1, 0);
      break;

    case 'S':
      controleManual(0, 0);
      break;

    case 'x':
      printOdom();
      break;
  }
}

void printOdom()
{
  Serial.println(x);
  Serial.println(y);
  Serial.println(theta);
  Serial.println();
}
