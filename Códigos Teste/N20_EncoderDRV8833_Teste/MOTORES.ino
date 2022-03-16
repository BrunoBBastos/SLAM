void direcao(char motE, char motD) {

  if (motE == 1) {
    digitalWrite(mL1, 1);
    digitalWrite(mL2, 0);
  }
  else if (motE == 0) {
    digitalWrite(mL1, 0);
    digitalWrite(mL2, 0);
  }
  else if (motE == -1) {
    digitalWrite(mL1, 0);
    digitalWrite(mL2, 1);
  }
  if (motD == 1) {
    digitalWrite(mR1, 0);
    digitalWrite(mR2, 1);
  }
  else if (motD == 0) {
    digitalWrite(mR1, 0);
    digitalWrite(mR2, 0);
  }
  else if (motD == -1) {
    digitalWrite(mR1, 1);
    digitalWrite(mR2, 0);
  }
}
