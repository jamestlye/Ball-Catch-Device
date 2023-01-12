int flexSensorPin = A0; //reading from analog pin 0

void setup() {
Serial.begin(9600);
}

void loop() {
 int flexSensorReading = analogRead(flexSensorPin);
 Serial.println(flexSensorReading);
 delay(1000); //delay monitor reading
}
