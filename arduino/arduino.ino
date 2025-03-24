void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  int sensorValue = analogRead(A0);  
  Serial.println(sensorValue); 

  if (Serial.available() > 0) {
    byte command = Serial.read();  
    if (command == 1) {
      digitalWrite(LED_BUILTIN, HIGH);  
    } else if (command == 0) {
      digitalWrite(LED_BUILTIN, LOW);   
    }
  }

  delay(1000); 
}
