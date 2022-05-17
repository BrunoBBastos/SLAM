
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
