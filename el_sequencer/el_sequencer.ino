#include <SoftwareSerial.h>

// TODO: this isn't working. i think our cables might be too long. nothing is coming in over this
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

// the teensy already has logic for keeping the lights from flickering. this is really just a backup
// TODO: delete this logic entirely? having it just on the teensy side should be fine
unsigned int minOnMs = 11.5 * 2;
unsigned long turnOffMsArray[NUM_OUTPUTS];

void setup() {
  Serial.begin(115200);

  mySerial.begin(9600);

  for (int i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(outputPins[i], OUTPUT);
  }

  Serial.println("Started...");
}

void loop() {
  if (mySerial.available()) {
    data = mySerial.read();

    lastData = millis();

    for (int i = 0; i < NUM_OUTPUTS; i++) {
      if (bitRead(data, i) == 1) {
        // TODO: make sure that we don't turn on more than 6 lights
        // TODO: I think we need to loop once to check if the lights should turn off and then loop again so that numOn is correct
        digitalWrite(outputPins[i], HIGH);

        // make sure we stay on for a minimum amount of time
        turnOffMsArray[i] = millis() + minOnMs;
        Serial.print("| 1 ");
      } else {
        if (millis() < turnOffMsArray[i]) {
          // the output has not been on for long enough. leave it on.
          Serial.print("| 0 ");
        } else {
          // the output has been on for at least minOnMs and is quiet now. turn it off
          digitalWrite(outputPins[i], LOW);
          Serial.print("|   ");
        }
      }
    }
    Serial.println("|");
  }

  if (millis() - lastData < 5000) {
    // while we wait for new data from the Teensy
    // check to see if we should turn anything off
    for (int i = 0; i < NUM_OUTPUTS; i++) {
      if (bitRead(data, i) == 0) {
        // this output should be off
        if (millis() < turnOffMsArray[i]) {
          // the output has not been on for long enough to prevent flickering. leave it on.
        } else {
          // the output has been on for at least minOnMs and is quiet now. turn it off
          // we do this every iteration to be as responsive as possible
          digitalWrite(outputPins[i], LOW);

          // flip the bit so we don't bother checking this output again
          bitSet(data, i);
        }
      }
    }
  } else {
    // it's been over 5 seconds and we haven't received any data, blink

    Serial.println("| BLINKY |");

    // TODO: actually blink a cool pattern
    digitalWrite(outputPins[0], HIGH);
    digitalWrite(outputPins[2], HIGH);
    digitalWrite(outputPins[4], HIGH);
    digitalWrite(outputPins[6], HIGH);
    delay(1000);
  }
}
