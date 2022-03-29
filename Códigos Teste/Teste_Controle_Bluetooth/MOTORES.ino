void direcao(char motE, char motD) {

  if (motE == 1) {
    digitalWrite(MLA, 1);
    digitalWrite(MLB, 0);
  }
  else if (motE == 0) {
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 0);
  }
  else if (motE == -1) {
    digitalWrite(MLA, 0);
    digitalWrite(MLB, 1);
  }
  if (motD == 1) {
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 1);
  }
  else if (motD == 0) {
    digitalWrite(MRA, 0);
    digitalWrite(MRB, 0);
  }
  else if (motD == -1) {
    digitalWrite(MRA, 1);
    digitalWrite(MRB, 0);
  }
}
