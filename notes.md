TODO:
- split into multiple ino files
- clean up these includes
- clean up these defines
- what pin is A2?
- move DEFAULT_BRIGHTNESS to the SD card with IniFile
- change numFreqBands to maxFreqBands and read a value off SD card
- change numOutputs to maxOutputs and read a value off SD card
- change numLEDs to maxLEDs and read a value off SD card
- get rid of outputs and outputsStretched. just put it from frequencyColors into leds
- read mosr of the hard coded variables as from the SD card
- replace Serial.* with DEBUG_*
- finish cleaning up mapFrequencyColorsToOutputs

TODO V2: 
- set minOnMs based on some sort of bpm detection. if that is too hard, have a button
- does it matter if elapsedMs overflows? is that even possible given that we have 3 AA batteries?
- what should we do if we can't calculate E?
- find the sum of all the frequencies as well as the bins at least 10% as loud as the loudest bins for extra details?
- spread it around so that only 75% of the visualizer is shown. the other 25% will be on other hats
