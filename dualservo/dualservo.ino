#include <Servo.h>
Servo motorServosatu; // servo bawah (x)
Servo motorServodua; // servo bawah (y)
// initial position 1,1

void setup() {
  motorServosatu.attach(8);
  motorServodua.attach(9);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int val1, val2;

    // Memisahkan nilai input menjadi dua bagian menggunakan tanda koma (,)
    int index = input.indexOf(',');
    if (index != -1) {
      val1 = input.substring(0, index).toInt();
      val2 = input.substring(index + 1).toInt();
      
      // Memastikan nilai input valid (0 <= nilai <= 180)
      if (val1 >= 0 && val1 <= 180 && val2 >= 0 && val2 <= 180) {
        motorServosatu.write(val1-2);
        motorServodua.write(val2-2);
        
        // Mengirimkan balasan ke Serial Monitor
        Serial.print("Motor 1: ");
        Serial.print(val1);
        Serial.print(", Motor 2: ");
        Serial.println(val2);
      }
    }
  }
}
