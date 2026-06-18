#include <SoftwareSerial.h>

// RX = Uno pin 10, TX = Uno pin 11
SoftwareSerial mySerial(10, 11);  // RX, TX

void setup() {
  Serial.begin(115200);     // USB Serial Monitor
  mySerial.begin(9600);     // Software UART to Pico RX
  Serial.println("Uno SoftwareSerial TX test...");
}

void loop() {
  mySerial.print("hello123\r");  // Send CR-terminated string
  Serial.println("Sent via SoftwareSerial: hello123");
  delay(1000);
}
