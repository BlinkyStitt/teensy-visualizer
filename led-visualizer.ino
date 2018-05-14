#include <stdlib.h>

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <SoftwareSerial.h>
#include <elapsedMillis.h>
#include <FastLED.h>

// Use these with the audio adaptor board
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  7
#define SDCARD_SCK_PIN   14
#define VOLUME_KNOB      A2

// audio adaptor board
#define VOLUME_KNOB       A2
// select device
#define AUDIO_CS_PIN      10
// master out, slave in
#define ALT_MOSI_PIN      7
// clock
#define ALT_SCK_PIN       14

#define LED_DATA_PIN       0  // TODO: what pin. i want to share with the SD card
#define LED_CLOCK_PIN      1  // TODO: what pin. i want to share with the SD card
#define LED_CHIPSET        APA102
#define LED_MODE           BGR
#define DEFAULT_BRIGHTNESS 50
#define FRAMES_PER_SECOND  120

AudioInputI2S             i2s1;           //xy=139,91
AudioOutputI2S            i2s2;           //xy=392,32
AudioAnalyzeFFT1024       fft1024;
AudioConnection           patchCord1(i2s1, 0, i2s2, 0);
AudioConnection           patchCord2(i2s1, 0, fft1024, 0);
AudioControlSGTL5000      audioShield;    //xy=366,225

elapsedMillis elapsedMs = 0;    // todo: do we care if this overflows?

const int numOutputs = 8;

// we don't want all the lights to be on at once (TODO: at least, this was true with the EL. might be different here since we have control over brightness)
int numOn = 0;
int maxOn = 5;

CRGB leds[numOutputs];

// EMA factor  // TODO: tune this. this worked for EL, but now I think we might tie the brightness to it. read from SD card
float decayAvg = 0.60;
// how close a sound has to be to the loudest sound in order to activate
// TODO: tune this! was .99 with EL, but LED should probably be different
float activateDifference = 0.99;
// simple % decrease
float decayMax = 0.98;
float minMaxLevel = 0.15 / activateDifference;

// arrays to keep track of the volume for each frequency band
// TODO: eventually track numOutputs *3 or something like that and then have the color shift to follow the loudest of the 3
float maxLevel[numOutputs];
float currentLevel[numOutputs];

// going through the levels loudest to quietest makes it so we can ensure the loudest get turned on ASAP
int sortedLevelIndex[numOutputs];

// keep track of when to turn lights off so that they don't flicker off if the sound drops
unsigned long turnOffMsArray[numOutputs];
unsigned long lastUpdate = 0;

// keep the lights from blinking too fast
uint minOnMs = 200; // 118? 150? 184? 200?  // the shortest amount of time to leave an output on. todo: set this based on some sort of bpm detection? read from the SD card? have a button to switch between common settings?

// https://forum.pjrc.com/threads/33390-FFT-convert-real-values-to-decibles
// TODO: use this for something?
float db(float n, float r) {
  // r is the dB reference (here better calibration) value and describes microphone sensitivity and all gains in the processing chain
  if (n <= 0) return r-96;  // or whatever you consider to be "off"
  return r+log10f(n) * 20.0f;
}

/* sort the levels normalized against their max
 *
 * with help from https://phoxis.org/2012/07/12/get-sorted-index-orderting-of-an-array/
 */
static int compare_levels(const void *a, const void *b)
{
  int aa = *((int *) a), bb = *((int *) b);
  return (currentLevel[bb] / maxLevel[bb]) - (currentLevel[aa] / maxLevel[aa]);
}

void setup() {
  Serial.begin(115200);  // todo: tune this

  // setup SPI for the Audio board (which has an SD card reader)
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);

  // slave select pin for SPI
  pinMode(SDCARD_CS_PIN, OUTPUT);

  SPI.begin();
  // TODO: read SD card here to configure things

  // TODO: read numOutputs from the SD card

  // TODO: clock select pin for FastLED to OUTPUT like we do for the SDCARD?
  FastLED.addLeds<LED_CHIPSET, LED_DATA_PIN, LED_CLOCK_PIN, LED_MODE>(leds, numOutputs);
  FastLED.setBrightness(DEFAULT_BRIGHTNESS);  // TODO: read this from the SD card
  FastLED.clear();
  FastLED.show();

  // Audio requires memory to work. I haven't seen this go over 11
  AudioMemory(12);

  // Enable the audio shield and set the output volume.
  audioShield.enable();
  audioShield.muteHeadphone(); // to avoid any clicks
  audioShield.inputSelect(AUDIO_INPUT_MIC);
  audioShield.volume(0.5);
  audioShield.micGain(60);  // was 63, then 40  // 0-63 // TODO: tune this

  audioShield.audioPreProcessorEnable();  // todo: pre or post?

  // bass, mid_bass, midrange, mid_treble, treble
  // TODO: tune this. maybe read from SD card
  audioShield.eqSelect(GRAPHIC_EQUALIZER);
  // audioShield.eqBands(-0.80, -0.75, -0.50, 0.50, 0.80);  // the great northern
  // audioShield.eqBands(-0.5, -.2, 0, .2, .5);  // todo: tune this
  audioShield.eqBands(-0.80, -0.10, 0, 0.10, 0.33);  // todo: tune this

  audioShield.unmuteHeadphone();  // for debugging

  // setup sorting
  for (int i = 0; i < numOutputs; i++) {
    sortedLevelIndex[i] = i;
  }

  Serial.println("Starting...");
}

// we could/should pass fft and level as args
void updateLevelsFromFFT() {
  // TODO: go numb to constant noise on a per-bin basis

  // read the 512 FFT frequencies into numOutputs levels
  // music is heard in octaves, but the FFT data
  // is linear, so for the higher octaves, read
  // many FFT bins together.

  // See this conversation to change this to more or less than 16 log-scaled bands
  // https://forum.pjrc.com/threads/32677-Is-there-a-logarithmic-function-for-FFT-bin-selection-for-any-given-of-bands

  // TODO: tune these. maybe write a formula, but i think tuning by hand will be prettiest
  // TODO: have numOutputs = numOutputs * 3 and then set the color from there
  switch(numOutputs) {
    case 0:
    case 1:
      // TODO: this doesn't look right on our line graph
      currentLevel[0]  = fft1024.read(0, 465);  // TODO: tune this 465 = 20k
      break;
    case 2:
      currentLevel[0]  = fft1024.read(1, 10);  // TODO: tune this
      currentLevel[1]  = fft1024.read(11, 82);  // TODO: tune this
      break;
    case 3:
      currentLevel[0]  = fft1024.read(1, 6);  // TODO: tune this
      currentLevel[1]  = fft1024.read(7, 10);  // TODO: tune this
      currentLevel[2]  = fft1024.read(11, 82);  // TODO: tune this
      break;
    case 4:
      currentLevel[0]  = fft1024.read(1, 6);  // TODO: tune this
      currentLevel[1]  = fft1024.read(7, 20);  // TODO: tune this
      currentLevel[2]  = fft1024.read(21, 41);  // TODO: tune this
      currentLevel[3]  = fft1024.read(42, 186);  // TODO: tune this
      break;
    case 5:
      currentLevel[0]  = fft1024.read(1, 6);  // TODO: tune this
      currentLevel[1]  = fft1024.read(7, 10);  // TODO: tune this
      currentLevel[2]  = fft1024.read(11, 20);  // TODO: tune this
      currentLevel[3]  = fft1024.read(21, 41);  // TODO: tune this
      currentLevel[4]  = fft1024.read(42, 186);  // TODO: tune this
      break;
    case 6:
      currentLevel[0]  = fft1024.read(1, 3);   // 110
      currentLevel[1]  = fft1024.read(4, 6);   // 220
      currentLevel[2]  = fft1024.read(7, 10);  // 440
      currentLevel[3]  = fft1024.read(11, 20);  // 880
      currentLevel[4]  = fft1024.read(21, 41);  // 1763
      currentLevel[5]  = fft1024.read(42, 186);  // 7998 todo: tune this
      break;
    case 7:
      currentLevel[0]  = fft1024.read(1, 3);  // TODO: tune this
      currentLevel[1]  = fft1024.read(4, 6);  // TODO: tune this
      currentLevel[2]  = fft1024.read(7, 10);  // TODO: tune this
      currentLevel[3]  = fft1024.read(11, 20);  // TODO: tune this
      currentLevel[4]  = fft1024.read(21, 41);  // TODO: tune this
      currentLevel[5]  = fft1024.read(42, 82);  // TODO: tune this
      currentLevel[6]  = fft1024.read(83, 186);  // TODO: tune this
      break;
    case 8:
      currentLevel[0]  = fft1024.read(1, 15);
      currentLevel[1]  = fft1024.read(16, 32);
      currentLevel[2]  = fft1024.read(33, 46);
      currentLevel[3]  = fft1024.read(47, 66);
      currentLevel[4]  = fft1024.read(67, 93);
      currentLevel[5]  = fft1024.read(94, 131);
      currentLevel[6]  = fft1024.read(132, 184);
      currentLevel[7]  = fft1024.read(185, 419);  // 18kHz
      break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    default:
      // these bands are from pjrc. we should try to have the other ones match this growth rate
      // TODO: maybe skip the top and bottom bins?
      currentLevel[0]  = fft1024.read(0);  // TODO: skip this bin?
      currentLevel[1]  = fft1024.read(1);
      currentLevel[2]  = fft1024.read(2, 3);
      currentLevel[3]  = fft1024.read(4, 6);
      currentLevel[4]  = fft1024.read(7, 10);
      currentLevel[5]  = fft1024.read(11, 15);
      currentLevel[6]  = fft1024.read(16, 22);
      currentLevel[7]  = fft1024.read(23, 32);
      currentLevel[8]  = fft1024.read(33, 46);
      currentLevel[9]  = fft1024.read(47, 66);
      currentLevel[10] = fft1024.read(67, 93);
      currentLevel[11] = fft1024.read(94, 131);
      currentLevel[12] = fft1024.read(132, 184);
      currentLevel[13] = fft1024.read(185, 257);
      currentLevel[14] = fft1024.read(258, 359);
      currentLevel[15] = fft1024.read(360, 465);   // 465 = 20k
      break;
    case 17:
    case 18:
    case 19:
    case 20:
    case 21:
    case 23:
    case 24:
      // TODO: properly fill this. if we do 3 possible colors for each output with 8 outputs, we need this. a formula is looking better
      currentLevel[0]  = fft1024.read(0);  // TODO: skip this bin?
      currentLevel[1]  = fft1024.read(1);
      currentLevel[2]  = fft1024.read(2, 3);
      currentLevel[3]  = fft1024.read(4, 6);
      currentLevel[4]  = fft1024.read(7, 10);
      currentLevel[5]  = fft1024.read(11, 15);
      currentLevel[6]  = fft1024.read(16, 22);
      currentLevel[7]  = fft1024.read(23, 32);
      currentLevel[8]  = fft1024.read(33, 46);
      currentLevel[9]  = fft1024.read(47, 66);
      currentLevel[10] = fft1024.read(67, 93);
      currentLevel[11] = fft1024.read(94, 131);
      currentLevel[12] = fft1024.read(132, 184);
      currentLevel[13] = fft1024.read(185, 257);
      currentLevel[14] = fft1024.read(258, 359);
      currentLevel[15] = fft1024.read(360, 465);
      currentLevel[16] = fft1024.read(360, 465);
      currentLevel[17] = fft1024.read(360, 465);
      currentLevel[18] = fft1024.read(360, 465);
      currentLevel[19] = fft1024.read(360, 465);
      currentLevel[20] = fft1024.read(360, 465);
      currentLevel[21] = fft1024.read(360, 465);
      currentLevel[22] = fft1024.read(360, 465);
      currentLevel[23] = fft1024.read(360, 465);   // 465 = 20k
      break;
  }
}

float getLocalMaxLevel(int i) {
    float localMaxLevel = maxLevel[i];

    if (i != 0) {
      // check previous level if we aren't the first level
      localMaxLevel = max(localMaxLevel, maxLevel[i - 1]);
    }

    if (i != numOutputs) {
      // check the next level if we aren't the last level
      localMaxLevel = max(localMaxLevel, maxLevel[i + 1]);
    }

    return localMaxLevel;
}

void loop() {
  // TODO: determine the note being played?
  // TODO: determine the tempo?
  // TODO: find the sum as well as the bins at least 10% as loud as the loudest bin IDs for extra details?

  if (fft1024.available()) {
    updateLevelsFromFFT();

    // turn off any quiet levels. we do this before turning any lights on so that our loudest frequencies are most responsive
    for (int i = 0; i < numOutputs; i++) {
      // update maxLevel
      // TODO: don't just track max. track the % change. then do something with stddev of neighbors?
      if (currentLevel[i] > maxLevel[i]) {
        maxLevel[i] = currentLevel[i];
      }

      // turn off if current level is less than the activation threshold
      if (currentLevel[i] < maxLevel[i] * activateDifference) {
        // the output should be off
        if (elapsedMs < turnOffMsArray[i]) {
          // the output has not been on for long enough to prevent flicker
          // leave it on but reduce brightness at the same rate we reduce maxLevel (TODO: tune this)
          // TODO: should the brightness be tied to the currentLevel somehow? that might make it too random looking
          // TODO: probably should be tied to minOnMs so that it fades to minimum brightness before turning off
          // using "video" scaling, meaning: never fading to full black
          leds[i].fadeLightBy(int((1.0 - decayMax) * 255));
        } else {
          // the output has been on for at least minOnMs and is quiet now
          // if it is on, turn it off
          if (leds[i]) {
            // TODO: this might be too abrupt
            leds[i] = CRGB::Black;
            numOn -= 1;
          }
        }
      }
    }

    // sort the levels normalized against their max
    // this allows us to prioritize turning on for the loudest sounds
    qsort(sortedLevelIndex, numOutputs, sizeof(float), compare_levels);

    // turn on up to maxOn loud levels in order of loudest to quietest
    for (int j = 0; j < numOutputs; j++) {
      int i = sortedLevelIndex[j];

      // TODO: also check the neighbor maxLevels, too
      if (currentLevel[i] >= maxLevel[i] * activateDifference) {
        // this light should be on!
        if (numOn >= maxOn) {
          // except we already have too many lights on! don't do anything since this light is already off
          // don't break the loop because we still want to decay max level and process other lights
        } else {
          // we have room for the light! turn it on

          // if it isn't already on, increment numOn
          if (! leds[i]) {
            // track to make sure we don't turn too many lights on. some configurations max out at 6.
            // we don't do this every time because it could have already been on, but now we made it brighter
            numOn += 1;
          }

          // map(value, fromLow, fromHigh, toLow, toHigh)
          // TODO: set color_hue based on subdividing this level
          int color_hue = map(i, 0, numOutputs - 1, 0, 255);
          // TODO: what should saturation be? maybe not 255
          // set 255 as the max brightness. if that is too bright, FastLED.setBrightness can be changed in setup

          // look at neighbors and use their max for brightness if they are louder (but don't be less than 25% on!)
          int color_value = max(64, int(currentLevel[i] / getLocalMaxLevel(i) * 255));

          // https://github.com/FastLED/FastLED/wiki/FastLED-HSV-Colors#color-map-rainbow-vs-spectrum
          // HSV makes it easy to cycle through the rainbow
          leds[i] = CHSV(color_hue, 255, color_value);

          // make sure we stay on for a minimum amount of time
          // if we were already on, extend the time that we stay on
          turnOffMsArray[i] = elapsedMs + minOnMs;
        }
      }

      // decay maxLevel
      maxLevel[i] = (decayMax * maxLevel[i]) + ((1 - decayMax) * minMaxLevel);
    }

    // debug print
    for (int i = 0; i < numOutputs; i++) {
      Serial.print("| ");

      // TODO: maybe do something with parity here? i think i don't have enough lights for that to matter at this point. do some research

      if (leds[i]) {
        //Serial.print(leds[i].getLuma() / 255.0);
        Serial.print(maxLevel[i]);
      } else {
        Serial.print("    ");
      }
    }
    Serial.print("| ");
    Serial.print(AudioMemoryUsageMax());
    Serial.print(" blocks | ");

    // display the colors
    FastLED.show();

    // finish debug print
    Serial.print(elapsedMs - lastUpdate);
    Serial.println("ms");
    lastUpdate = elapsedMs;
    Serial.flush();

    // using FastLED's delay allows for dithering
    // we sleep for a while inside the loop since we know we don't need to process anything for 11 or 12 ms
    FastLED.delay(10);
  }

  // using FastLED's delay allows for dithering
  // a longer sleep is inside the fft available loop
  FastLED.delay(1);
}

