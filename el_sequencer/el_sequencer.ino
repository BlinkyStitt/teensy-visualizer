// TODO: next time, I think we should just use the shield instead of having two MCUs

#include <SoftwareSerial.h>

// TODO: why didn't A6 and A7 work?
#define SEQUENCER_RX A4  // green connected to Teensy's TX (1)
#define SEQUENCER_TX A3  // yellow connected to Teensy's RX (0)
SoftwareSerial mySerial(SEQUENCER_RX, SEQUENCER_TX);

#define NUM_OUTPUTS 8  // 1-8
#define OUTPUT_A 2
#define OUTPUT_B 3
#define OUTPUT_C 4
#define OUTPUT_D 5
#define OUTPUT_E 6
#define OUTPUT_F 7
#define OUTPUT_G 8
#define OUTPUT_H 9
const int outputPins[NUM_OUTPUTS] = {OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, OUTPUT_E, OUTPUT_F, OUTPUT_G, OUTPUT_H};

// data from the Teensy maps to the 8 outputs
unsigned char data = 0;

// track the millis() of when we last received a message over serial
unsigned long lastData = 0;

void setup() {
  Serial.begin(115200);

  mySerial.begin(9600);

  for (int i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(outputPins[i], OUTPUT);

    // blink the wire once
    digitalWrite(outputPins[i], HIGH);
    delay(200);
    digitalWrite(outputPins[i], LOW);
  }

  Serial.println("Started...");
}

void loop() {
  if (mySerial.available()) {
    data = mySerial.read();

    lastData = millis();

    // turn the outputs on or off depending on the data received
    for (int i = 0; i < NUM_OUTPUTS; i++) {
      if (bitRead(data, i) == 1) {
        Serial.print("| 1 ");
        digitalWrite(outputPins[i], HIGH);
      } else {
        Serial.print("|   ");
        digitalWrite(outputPins[i], LOW);
      }
    }
    Serial.println("|");
  }

  if (millis() - lastData > 5000) {
    // it's been over 5 seconds and we haven't received any data, blink fun patterns
    while (1) {
      if (mySerial.available()) {
        // the serial connection has returned!
        // i doubt this will ever happen, but it doesn't really hurt to check
        break;
      }

      blinkPatterns();
    }
  }
}

void blinkPatterns() {
  Serial.println("| BLINKY |");

  // TODO: actually blink a cool pattern
  digitalWrite(outputPins[0], HIGH);
  digitalWrite(outputPins[2], HIGH);
  digitalWrite(outputPins[4], HIGH);
  digitalWrite(outputPins[6], HIGH);
  delay(1000);
}
