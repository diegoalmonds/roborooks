// MANUAL TESTING
// const int magnetPin = 9;

// void setup() {
//   pinMode(magnetPin, OUTPUT);
//   digitalWrite(magnetPin, LOW);
//   Serial.begin(115200);
//   // Serial.println("Ready! Press 1 to turn ON, 0 to turn OFF");
// }

// void loop() {
//   if (Serial.available() > 0) {
//     char key = Serial.read();
    
//     if (key == '1') {
//       digitalWrite(magnetPin, HIGH);
//       Serial.println("Magnet ON");
//     }
    
//     if (key == '0') {
//       digitalWrite(magnetPin, LOW);
//       Serial.println("Magnet OFF");
//     }
//   }
// }

// loaded program that works directly with robot
void setup() { 
  Serial.begin(115200);
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  Serial.println("Received: " + cmd);

  if (cmd.startsWith("PIN ")) {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    int pin = cmd.substring(firstSpace + 1, secondSpace).toInt();
    String state = cmd.substring(secondSpace + 1);
    state.trim();

    Serial.println("Pin: " + String(pin));
    Serial.println("State: " + state);

    pinMode(pin, OUTPUT);
    digitalWrite(pin, (state == "ON") ? HIGH : LOW);
  }
}