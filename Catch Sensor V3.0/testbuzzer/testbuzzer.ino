
int buzzer = 11;
void setup() {


}

void loop() {

  // no need to repeat the melody.
  tone(buzzer,3000,125);
  delay(125);
  tone(buzzer,4000,125);
  delay(2000);
  tone(buzzer,4000,125);
  delay(125);
  tone(buzzer,3000,125);
  delay(4000);
}
